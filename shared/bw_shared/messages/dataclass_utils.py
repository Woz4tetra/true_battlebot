from __future__ import annotations

import dataclasses
from collections.abc import MutableMapping
from dataclasses import asdict
from datetime import datetime
from enum import Enum
from inspect import isclass
from typing import (
    Any,
    Callable,
    Dict,
    FrozenSet,
    Generic,
    Iterable,
    Iterator,
    List,
    Literal,
    Mapping,
    Optional,
    Protocol,
    Set,
    Tuple,
    Type,
    TypeVar,
    Union,
    _GenericAlias,  # type: ignore
    cast,
    get_type_hints,
    runtime_checkable,
)

import dacite
import marshmallow
import numpy as np
from dacite import Config
from dacite.types import extract_generic, extract_optional, is_literal, is_optional, is_union
from marshmallow.validate import Length
from marshmallow_polyfield import PolyField, PolyFieldBase

# TODO app/shared/dataclass_utils now has a DataclassType typehint; use it in this code.
# While you're at it you might as well replace direct inspection of __dataclass_fields__ with use of
# the dataclasses.fields() function.

# PolyFieldBase is a superclass of PolyField; use it for typehints
MarshmallowField = Union[PolyFieldBase, Type[marshmallow.fields.Field]]

SchemaType = marshmallow.schema.SchemaMeta

T = TypeVar("T")

T_co = TypeVar("T_co", covariant=True)


class AmbiguousUnionError(marshmallow.exceptions.ValidationError):
    """
    Indicates that the data could match to any of a number of possible types
    """

    def __init__(self, message: List[Type], data: Any):
        super().__init__(message={self.__class__.__name__: message}, data=data)


class UnknownTypeUnionError(marshmallow.exceptions.ValidationError):
    """
    Indicates that the requested type specified doesn't exist
    """

    def __init__(self, requested_type: str, valid_types: Iterable[Type], data: Any):
        message = f"Unknown type `{requested_type}`, expected one of {valid_types}"
        super().__init__(message, data=data)


def create_union_error(
    type_errs: Dict[Type, marshmallow.exceptions.ValidationError], data: Any
) -> marshmallow.exceptions.ValidationError:
    requested_type = data["type"]

    all_type_errors = all("type" in errs.messages for errs in type_errs.values())
    if all_type_errors:
        all_type_names = [type_.type for type_ in type_errs.keys()]
        # TODO: Maybe just return the section of data in question?
        return UnknownTypeUnionError(requested_type, all_type_names, data)
    else:
        type_ = next(type_ for type_ in type_errs.keys() if type_.type == requested_type)
        return type_errs[type_]


class AnyLengthTuple(marshmallow.fields.Tuple):
    """
    A variant of marshmallow.fields.Tuple that can handle Tuple[foo, ...]
    """

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        assert len(self.tuple_fields) == 1, "Must have exactly 1 type for variable length tuples"
        self.field_type = self.tuple_fields[0]

    def _deserialize(self, value, attr, data, **kwargs) -> tuple:  # type: ignore
        """
        Force self.tuple_fields to match the length of `value` before deserializing
        """
        self.tuple_fields = [self.field_type] * len(value)
        self.validate_length = Length(equal=len(self.tuple_fields))
        return super()._deserialize(value, attr, data, **kwargs)


class EnumSchema(marshmallow.fields.Field):
    """
    Deserialization schema for Enum subclasses
    """

    def __init__(self, cls: Type, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        assert issubclass(cls, Enum)
        self.enum_class = cls

    def _deserialize(self, value: Any, attr: Optional[str], data: Optional[Mapping[str, Any]], **kwargs: Any) -> Any:
        """
        Find the Enum field with matching value
        """
        try:
            return self.enum_class(value)
        except ValueError:
            raise marshmallow.ValidationError(f"{self.enum_class} has no field for `{value}`")


# because of how mypy and dacite's internals work, is_union doesn't think that Literals are Unions
def is_union_or_literals(type_: Type) -> bool:
    return is_union(type_) or (is_literal(type_) and len(type_.__args__) > 1)


class DataclassConversionError(NotImplementedError):
    def __init__(self, message: Any):
        self.message = "Dataclass field type {} cannot be converted to a marshmallow field type.".format(str(message))


class UnionConversionError(NotImplementedError):
    def __init__(self, member: Any, union: Type[T]):
        self.message = "Cannot make schema for member type {} of Union {}".format(member, union)


def type_to_marshmallow_constructor(  # noqa: C901
    t: type,
) -> Tuple[MarshmallowField, Dict[str, Any]]:
    """
    Takes a type (or a typing library type hint - eg Tuple[int, str]; Literal["name"], List[Foo])
    and returns a tuple of a marshmallow field constructor and a set of keyword arguments to it
    which are definitely required.

    Doesn't just construct a marshmallow field all in one go because sometimes other keyword args
    should be applied to the marshmallow constructor before building the marshmallow field (eg.
    optionality information).
    """
    constructor_kwargs: Dict[str, Any] = {}

    # Map Python datatypes to Marshmallow field constructors.
    base_types_to_marshmallow_constructors = {
        bool: marshmallow.fields.Bool,
        int: marshmallow.fields.Int,
        float: marshmallow.fields.Float,
        str: marshmallow.fields.Str,
    }

    # Sometimes target types are subclasses of typing types. For example, we have the Coord3D,
    # DegreeQ, and RadianQ classes, which all subclass typing.Tuple. In this case, we pretend that
    # the target type is just the typing type that is being subclassed.
    # See https://github.com/python/typeshed/issues/3983 about the type: ignore
    if isclass(t) and issubclass(t, Generic):  # type: ignore
        # Only tuples, lists, and dicts are supported so far...
        if issubclass(t, tuple) or issubclass(t, dict) or issubclass(t, list):
            # Extract the typing module Tuple[foo, foo, ...] or Dict[foo, foo] instance from the
            # generic type bases.
            # To keep things sane, make sure that the class we're targeting only inherits from one
            # typing module Generic - it wouldn't make sense to deal with something that subclasses
            # more than one.
            # We don't get here if t is not a subclass of a Generic type, which means t will have
            # this field, but mypy doesn't understand, hence the type: ignore.
            bases = t.__orig_bases__  # type: ignore
            parents = [i for i in filter(lambda generic: isinstance(generic, _GenericAlias), bases)]
            assert len(parents) == 1
            parent = parents[0]  # This will look like eg. typing.Tuple[foo, foo, ...]
            assert parent.__origin__ in (tuple, dict, list)
            # Use the type we subclass from for the rest of the type-munging.
            t = parent
        # ...but conversions of other subclasses of typing module types is not supported yet.
        else:
            raise DataclassConversionError(t)

    # Figure out which marshmallow.Field constructor to use for this dataclass field.
    # - Resolution of base python types to marshmallow constructors is supported.
    if t in base_types_to_marshmallow_constructors:
        marshmallow_constructor = base_types_to_marshmallow_constructors[t]

    # - So are some typing library types, such as...:
    elif isinstance(t, _GenericAlias):
        # - ...conversions of typing_extensions.Literal[foo] to marshmallow Constant fields.
        if is_literal(t):
            if len(t.__args__) > 1:
                # If we're dealing with a Literal[foo, bar, ...] then we're actually dealing with
                # a Union; treat it like one.
                return union_to_polyfield(t)
            # Gets 'foo' out of typing_extensions.Literal[foo]
            # NOTE, when we use Python 3.8, we can use typing.getargs here instead.
            literal_value = t.__args__[0]
            literal_type = type(literal_value)
            if literal_type not in base_types_to_marshmallow_constructors:
                raise NotImplementedError(f"No current support for Literal[{literal_type}]")
            marshmallow_constructor = base_types_to_marshmallow_constructors[literal_type]
            constructor_kwargs["validate"] = marshmallow.validate.Equal(literal_value)

        # - ...conversions of typing.Lists to marshmallow List fields.
        elif t.__origin__ is list:
            marshmallow_constructor = marshmallow.fields.List
            list_element_type = t.__args__[0]
            # Typechecking on eg the foo in List[foo]
            if list_element_type:
                if dataclasses.is_dataclass(list_element_type):
                    list_element_schema = dataclass_to_schema(list_element_type)()
                    constructor_kwargs["cls_or_instance"] = marshmallow.fields.Nested(list_element_schema)
                elif is_union_or_literals(list_element_type):
                    polyfield, polyfield_kwargs = union_to_polyfield(list_element_type)
                    constructor_kwargs["cls_or_instance"] = polyfield(**polyfield_kwargs)
                else:
                    list_element_constructor, list_element_kwargs = type_to_marshmallow_constructor(list_element_type)
                    constructor_kwargs["cls_or_instance"] = list_element_constructor(**list_element_kwargs)
            else:
                # Type of List's elements not specified. Do no checking on them.
                constructor_kwargs["cls_or_instance"] = marshmallow.fields.Raw()

        # - ...conversion of typing.Dicts to marshmallow Dict fields.
        elif t.__origin__ is dict:
            marshmallow_constructor = marshmallow.fields.Dict
            key_type, value_type = t.__args__
            if isinstance(key_type, TypeVar) and isinstance(value_type, TypeVar):
                # Types of dict's keys, values not specified.
                constructor_kwargs["keys"] = None
                constructor_kwargs["values"] = None
            else:
                if dataclasses.is_dataclass(key_type):
                    key_schema = dataclass_to_schema(key_type)()
                    constructor_kwargs["keys"] = marshmallow.fields.Nested(key_schema)
                elif is_union_or_literals(key_type):
                    polyfield, polyfield_kwargs = union_to_polyfield(key_type)
                    constructor_kwargs["keys"] = polyfield(**polyfield_kwargs)
                else:
                    key_constructor, key_kwargs = type_to_marshmallow_constructor(key_type)
                    constructor_kwargs["keys"] = key_constructor(**key_kwargs)
                if dataclasses.is_dataclass(value_type):
                    value_schema = dataclass_to_schema(value_type)()
                    constructor_kwargs["values"] = marshmallow.fields.Nested(value_schema)
                elif is_union_or_literals(value_type):
                    polyfield, polyfield_kwargs = union_to_polyfield(value_type)
                    constructor_kwargs["values"] = polyfield(**polyfield_kwargs)
                else:
                    value_constructor, value_kwargs = type_to_marshmallow_constructor(value_type)
                    constructor_kwargs["values"] = value_constructor(**value_kwargs)

        # - ...conversion of typing.Tuple to marshmallow Tuple fields.
        elif t.__origin__ is tuple:
            marshmallow_constructor = marshmallow.fields.Tuple
            tuple_fields: List[MarshmallowField] = []
            element_types = t.__args__
            if (len(element_types) == 2) and (element_types[1] == Ellipsis):
                # Special case for Typle[foo, ...]
                marshmallow_constructor = AnyLengthTuple
                element_types = [element_types[0]]
                ...
            # Typechecking on eg. the foo in Tuple[foo, foo]
            if element_types:
                for element_type in element_types:
                    if dataclasses.is_dataclass(element_type):
                        element_schema = dataclass_to_schema(element_type)()
                        tuple_fields.append(marshmallow.fields.Nested(element_schema))
                    elif is_union_or_literals(element_type):
                        polyfield, polyfield_kwargs = union_to_polyfield(element_type)
                        tuple_fields.append(polyfield(**polyfield_kwargs))
                    else:
                        element_constructor, element_kwargs = type_to_marshmallow_constructor(element_type)
                        tuple_fields.append(element_constructor(**element_kwargs))
                constructor_kwargs["tuple_fields"] = tuple(tuple_fields)

        # - ...but we don't support other typing library cases.
        else:
            raise DataclassConversionError(t)
    elif t is type(None):  # noqa: E721
        marshmallow_constructor = marshmallow.fields.Raw
        constructor_kwargs["validate"] = marshmallow.validate.Equal(None)
        constructor_kwargs["allow_none"] = True
    elif issubclass(t, Enum):
        marshmallow_constructor = EnumSchema
        constructor_kwargs["cls"] = t
    else:
        raise DataclassConversionError(t)

    return marshmallow_constructor, constructor_kwargs


def dc_field_to_marshmallow_field(
    dc_field: dataclasses.Field, dc_field_type: type, validate: Optional[Callable]
) -> MarshmallowField:
    """
    Convert a dataclass Field to a marshmallow Field.

    This function is used to generate marshmallow schemas and should get called on the "leaf nodes"
    of nested dataclasses.
    """
    # Arguments to be given to the marshmallow Field at construction time.
    if is_union_or_literals(dc_field_type):
        marshmallow_constructor, constructor_kwargs = union_to_polyfield(dc_field_type)
    else:
        marshmallow_constructor, constructor_kwargs = type_to_marshmallow_constructor(dc_field_type)

    required, dump_default = dc_field_required(dc_field)
    constructor_kwargs["required"] = required
    if validate is not None:
        constructor_kwargs["validate"] = validate
    if dump_default is not None:
        constructor_kwargs["dump_default"] = dump_default

    return cast(MarshmallowField, marshmallow_constructor(**constructor_kwargs))


def make_polyfield_deser_func(  # noqa: C901
    union_members: tuple,
    members_to_schemas: Dict[type, Union[SchemaType, MarshmallowField]],
    field_type: type,
) -> Callable[[Any, Dict], type]:
    def _schema_deserialization_disambiguation(obj: Any, parent_object_dict: Dict) -> type:
        """
        Find which schema should be used to deserialize the obj by validating each option
        """
        # all of the types for which obj would be a valid input
        matching_types = []
        # mapping from each union member to associated validation errors
        errs = {}
        for member in union_members:
            mapped_schema = members_to_schemas[member]
            if isinstance(mapped_schema, marshmallow.fields.Field):
                try:
                    mapped_schema.deserialize(obj)
                except marshmallow.exceptions.ValidationError as validation_err:
                    errs[member] = validation_err
                else:
                    matching_types.append(member)
            else:
                # Need to instantiate a Schema
                schema = mapped_schema()
                assert isinstance(schema, marshmallow.Schema)
                if validation_errs := schema.validate(obj):
                    errs[member] = marshmallow.exceptions.ValidationError(validation_errs, data=obj)
                else:
                    matching_types.append(member)
        if not matching_types:
            raise create_union_error(errs, obj)
        if len(matching_types) > 1:
            raise AmbiguousUnionError(message=matching_types, data=obj)
        return members_to_schemas[matching_types[0]]

    return _schema_deserialization_disambiguation


def make_polyfield_ser_func(
    members_to_schemas: Dict[type, Union[SchemaType, MarshmallowField]],
) -> Callable[[Any, Any], MarshmallowField]:
    # NB Pretty sure this function is not used in config validation, so it may not actually work.
    def _schema_serialization_disambiguation(base_object: Any, parent_object: Any) -> MarshmallowField:
        try:
            return members_to_schemas[base_object.__class__]()
        except KeyError:
            pass

        raise TypeError(
            "Problem serializing - this code has never been run or tested before. "
            "base_object: {}; parent_object: {}".format(base_object, parent_object)
        )

    return _schema_serialization_disambiguation


def union_to_polyfield(union_: Type[Any]) -> Tuple[PolyFieldBase, Dict[str, Any]]:
    """
    Takes a union and returns a tuple of marshmallow PolyField (the class PolyField itself, to be
    used as a constructor), and a set of keyword arguments to it which are definitely required.

    Doesn't just construct a marshmallow PolyField all in one go because sometimes other keyword
    args should be applied to the PolyField constructor before building the PolyField (eg.
    optionality information).
    """
    assert is_union_or_literals(union_)

    # If 'union_' is a Literal['foo', 'bar', ...] convert it to a
    # Union[Literal['foo'], Literal['bar']...], which is necessary because of typing library
    # implementation choices. (We shouldn't have to mess with using GenericAlias directly like
    # this, but they basically made us.)
    if is_literal(union_) or (is_optional(union_) and is_literal(extract_optional(union_))):
        literals_extracted = tuple((Literal[t] for t in union_.__args__))
        union = Union[literals_extracted]  # type: ignore
    else:
        union = union_  # type: ignore
    constructor_kwargs: Dict[str, Any] = {}

    # Based on the different members of the Union, create ser/deser schema selectors.
    # See the marshmalow_polyfield README.
    union_members: tuple = extract_generic(union)  # type: ignore

    # Map members of the union to generated marshmallow.Schemas which verify them.
    members_to_schemas: Dict[type, Union[SchemaType, MarshmallowField]] = {}
    for member in union_members:
        if dataclasses.is_dataclass(member):
            members_to_schemas[member] = dataclass_to_schema(member)
        else:
            # For eg. the Int in Union[Foo, Int].
            try:
                constructor, kwargs = type_to_marshmallow_constructor(member)
                members_to_schemas[member] = constructor(**kwargs)
            except Exception:
                raise UnionConversionError(member, union)  # type: ignore

    # Use a the factory functions make_polyfield_ser_func and make_polyfield_deser_func
    # to make the ser/deser methods for these PolyFields so that their view of
    # members_to_schemas and union_members don't get trampled by redefinitions.
    constructor_kwargs["serialization_schema_selector"] = make_polyfield_ser_func(members_to_schemas)
    constructor_kwargs["deserialization_schema_selector"] = make_polyfield_deser_func(
        union_members, members_to_schemas, union_
    )
    # Use whether the original union was optional (not any re-packed union of Literals)
    constructor_kwargs["allow_none"] = is_optional(union_)

    # If we're dealing with a union of Literals, validate that the input is one of the allowed
    # literal values.
    if all([is_literal(m) for m in union_members]):
        values = [m.__args__[0] for m in union_members]
        constructor_kwargs["validate"] = marshmallow.validate.OneOf(values)
    return PolyField, constructor_kwargs


def dc_field_required(dc_field: dataclasses.Field) -> Tuple[bool, Optional[Any]]:
    """
    If a given dataclass field is required in the schema, returns (True, None); if the field is
    not required, returns (False, <the defualt value>).
    """
    if not dc_field.init:
        # This value does not get initialized on dataclass instantiation (it's probably calculated
        # in another method, like post_init), so it shouldn't be required in the schema either.
        return (False, None)
    elif not isinstance(dc_field.default, dataclasses._MISSING_TYPE):
        # Default value supplied; this field is optional.
        return (False, dc_field.default)
    # See https://github.com/python/typeshed/issues/3983 about the type: ignore
    elif not isinstance(dc_field.default_factory, dataclasses._MISSING_TYPE):
        # Default factory supplied; this field is optional.
        return (False, dc_field.default_factory())
    else:
        # No default value supplied; this is field is required.
        return (True, None)


def dataclass_to_schema(dataclass: Type[T]) -> SchemaType:  # noqa: C901
    """
    A recursive function which builds a Marshmallow Schema for validating Dacite dataclasses. It
    assumes that the passed in dataclass is a nested collection of base Python types, more
    dataclasses or typing.Unions of dataclasses or base Python types.

    After the returned Schema has been instantiated, its .load() method may be used on a
    dictionary object to validate the data within.

    Schema generation is done programatically here rather than by making developers write one
    boilerplatey Schema class per dataclass.
    """
    assert dataclasses.is_dataclass(dataclass)
    dc_fields: Dict[dataclasses.Field] = dataclass.__dataclass_fields__  # type: ignore
    schema_dict: Dict[str, MarshmallowField] = {}
    for field_name in dc_fields:
        dc_field: dataclasses.Field = dc_fields[field_name]

        if not dc_field.init:
            # If the field does not get initialized on instantiation, it is probably calculated
            # in the post_init method. The user should not be setting it, so don't add it to the
            # validation schema.
            continue
        field_type = get_type_hints(dataclass)[field_name]

        if dataclasses.is_dataclass(field_type):
            required, dump_default = dc_field_required(dc_field)

            schema = dataclass_to_schema(field_type)
            if dump_default is None:
                dump_default = marshmallow.missing

            schema_dict[field_name] = marshmallow.fields.Nested(schema, required=required, dump_default=dump_default)

        else:
            validate = None
            # Add a leaf node to the nested schema.
            try:
                wrapped_field_validator = dataclass.__dict__[f"validate_{field_name}"]

                if isinstance(wrapped_field_validator, staticmethod):
                    # __func__ unwraps the function from the staticmethod and makes it callable
                    # without the surrounding class.
                    validate = wrapped_field_validator.__func__
                else:
                    raise Exception(
                        f'Validation function "validate_{field_name}" specified ' "but it is not a @staticmethod"
                    )
            except KeyError:
                # No validator specified
                pass

            schema_dict[field_name] = dc_field_to_marshmallow_field(dc_field, field_type, validate)

    # Return the schema for this dataclass
    return cast(SchemaType, marshmallow.Schema.from_dict(schema_dict))


@runtime_checkable
class DaciteFriendly(Generic[T_co], Protocol):
    """
    A class which wraps types to basically add a second type annotation which indicates that that
    type implements a dacite_constructor method.

    If a wrapped class implements this dacite_constructor method, then pickle_dacite_config can use
    it to figure out how to convert a dictionary representation into an instance of that class, by
    taking advantage of dacite type hooks (see DaciteTypeHooks below).

    This can be used for classes which are too complicated for dacite to parse a dict into using
    dacite.Config.cast.
    """

    @staticmethod
    def dacite_constructor(dict_repr: Dict[Any, Any]) -> DaciteFriendly[T_co]: ...


class DaciteTypeHooks(MutableMapping):
    """
    Acts like a dict, which maps types to callables which contruct intances of that type.

    Types may be registered explicitly as you would normally insert something into a dictionary, but
    if you try to get a key which isn't in the dictionary but which implements DaciteFriendly, you
    will use its dacite_constructor instead. (This essentially lets the TypeHooks dict be as large
    as the number of DaciteFriendly classes.)
    """

    def __init__(self, *args: Any, **kwargs: Any):
        self.__dict__.update(*args, **kwargs)

    def __getitem__(self, key: Any) -> Any:
        if key in self.__dict__:
            return self.__dict__[key]
        elif isclass(key) and issubclass(key, DaciteFriendly):
            return key.dacite_constructor
        else:
            raise KeyError(f"Can't look up key '{key}'")

    def __setitem__(self, key: Any, value: Callable[[Any], Any]) -> None:
        self.__dict__[key] = value

    def __delitem__(self, key: Any) -> None:
        if key in self.__dict__:
            del self.__dict__[key]
        else:
            raise IndexError(f"Can't delete unstored key '{key}'")

    def __len__(self) -> int:
        """
        Kind of doesn't make sense for this mapping, but we return the number of explicitly set
        keys.
        """
        return len(self.__dict__)

    def __iter__(self) -> Iterator[Any]:
        """
        Kind of doesn't make sense for this mapping, but we iterate over the explicitly set keys.
        """
        return iter(self.__dict__)


def from_dict(cls: Type[T], data: dict) -> T:
    # Create a marshmallow schema from the dataclass
    schema = dataclass_to_schema(cls)()
    # The schema will validate the data and do some datatype conversions
    validated_data = schema.load(data)
    # dacite will instantiate a cls obj from the validated data
    inst = dacite.from_dict(
        cls,
        validated_data,
        Config(
            strict=True,
            type_hooks=DaciteTypeHooks(
                {
                    np.ndarray: np.array,
                    datetime: lambda x: datetime.fromisoformat(x),
                    FrozenSet: frozenset,
                    Set: set,
                }
            ),  # type: ignore
            cast=[Tuple, tuple, Enum, FrozenSet, Set],  # type: ignore
        ),
    )
    return inst


def _asdict_factory(data):
    def convert_value(obj):
        if isinstance(obj, Enum):
            return obj.value
        return obj

    return dict((k, convert_value(v)) for k, v in data)


def to_dict(obj) -> dict:
    return asdict(obj, dict_factory=_asdict_factory)
