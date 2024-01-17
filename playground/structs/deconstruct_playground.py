import deconstruct as c


class InputEvent(c.Struct):
    time: c.uint64[2]
    type: c.int16
    code: c.int16
    value: c.int32


buffer = b"Some  arbitrary  buffer!"
event = InputEvent(buffer)
assert event.code == 26229
assert event.time == (8241904116577431379, 2340027244253309282)
print(event)

print(InputEvent.new(123, 456, 789, 101112))
