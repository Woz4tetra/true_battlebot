"""
Code that creates nested `dataclass` instances tree from JSON data.
Somewhat similar to what Pydantic, Schematics do out of the box.
but native Python dataclasses don't do automatically.

See discussion here for context
https://stackoverflow.com/questions/51564841/creating-nested-dataclass-objects-in-python

See tests on the bottom of this file usage examples.
Tested on Python 3.9

=============== License =================

Copyright 2020 Daniel Dotsenko, daniel.dotsenko@panoramichq.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
documentation files (the "Software"), to deal in the Software without restriction, including without limitation the
rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit
persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the
Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

=========================================
(^ MIT license)
"""

from dataclasses import dataclass, fields, is_dataclass
from typing import Dict, List, Optional, Union
from unittest import TestCase
from unittest import main as run_tests


class NotSet:
    pass


def origin_type(field_type):
    return getattr(field_type, "__origin__", None)


def type_args(field_type):
    return getattr(field_type, "__args__", [])


def is_optional(field_type):
    return origin_type(field_type) is Union and type_args(field_type) and type_args(field_type)[-1] is type(None)


def unpack_optional(field_type):
    return type_args(field_type)[0]


def dataclass_deserialize(cls, data):
    """
    :param cls: A class, subtype of type dataclass
    :param data: dict with data to apply to the class when instantiated
    :return:
    """
    if is_dataclass(data) or not is_dataclass(cls):
        return data

    object_fields = fields(cls)
    tx_data = {}
    for field in object_fields:
        field_type = field.type
        name = field.name

        is_opt = is_optional(field_type)
        if is_opt:
            field_type = unpack_optional(field_type)

        if is_opt:
            value = data.get(name, NotSet)
        else:
            # Intended to throw Key exception.
            # If you catch, repackage into another exception.
            value = data[name]

        if value is NotSet:
            continue  # for loop

        if is_dataclass(field_type):
            tx_data[name] = dataclass_deserialize(field_type, value)
            continue  # for loop

        if origin_type(field_type) is list:
            field_type = type_args(field_type)[0]
            tx_data[name] = [dataclass_deserialize(field_type, e) for e in value]
            continue  # for loop

        if origin_type(field_type) is dict:
            _, field_type = type_args(field_type)
            tx_data[name] = {k: dataclass_deserialize(field_type, e) for k, e in value.items()}
            continue  # for loop

        # it's not a dataclass or a simple struct of them.
        # must some literal value
        # or some incomprehensible type that we don't yet care to parse, not our problem
        tx_data[name] = value

    return cls(**tx_data)


class DataclassRecursiveTests(TestCase):
    def test_works(self):
        @dataclass(frozen=True, eq=True)
        class A:
            x: str
            y: Optional[str] = None

        @dataclass(frozen=True, eq=True)
        class C:
            z: A

        @dataclass(frozen=True, eq=True)
        class B:
            c: C
            aa: List[A]
            ad: Dict[str, A]

            vs: str
            vi: int
            vl: List[int]
            vd: Dict[str, int]

            ao: Optional[A] = None
            ao2: Optional[A] = None

        data = dict(
            c=dict(z=dict(x="one", y="two")),
            aa=[
                dict(x="tree"),
                dict(x="four"),
            ],
            ad={"ASDF": dict(x="five"), "QWER": dict(x="six")},
            ao=dict(x="seven"),
            vs="eight",
            vi=9,
            vl=[10, 11],
            vd={"ZXCV": 12},
        )

        should_be = B(
            c=C(z=A(x="one", y="two")),
            aa=[
                A(x="tree"),
                A(x="four"),
            ],
            ad={"ASDF": A(x="five"), "QWER": A(x="six")},
            ao=A(x="seven"),
            # ao2 = null
            vs="eight",
            vi=9,
            vl=[10, 11],
            vd={"ZXCV": 12},
        )

        it_is = dataclass_deserialize(B, data)
        assert it_is == should_be

    def test_key_error(self):
        @dataclass(frozen=True, eq=True)
        class A:
            x: str
            y: Optional[str] = None

        @dataclass(frozen=True, eq=True)
        class B:
            a: A
            ao: Optional[A] = None

        data = dict(
            # a = dict(x='Not provided mandatory attr. should blow up'),
            ao=dict(x="seven"),
        )

        with self.assertRaises(KeyError) as expecting:
            dataclass_deserialize(B, data)
        error_message = str(expecting.exception)
        assert error_message == "'a'"


if __name__ == "__main__":
    run_tests()
