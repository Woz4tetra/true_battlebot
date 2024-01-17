from ctypes import c_uint8, c_uint32

from pyembc import pyembc_struct, pyembc_union


@pyembc_struct
class Inner:
    a: c_uint8
    b: c_uint8


@pyembc_struct
class Outer:
    first: Inner
    second: c_uint8


@pyembc_union
class MyUnion:
    as_struct: Outer
    as_int: c_uint32


inner = Inner()
print(inner)

inner = Inner(a=1, b=2)
print(inner)

# inner = Inner(a=256, b=300)

outer = Outer()
print(outer)

outer = Outer(first=Inner(a=1, b=2), second=3)
print(outer)


my_union = MyUnion()
print(my_union)

my_union = MyUnion(as_struct=Outer(first=Inner(a=1, b=2), second=3))
print(my_union)
