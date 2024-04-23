from ctypes import c_byte, c_char_p, cast


def fill_string(string: str, max_length: int) -> bytes:
    return string.encode() + b"\0" * (max_length - len(string))


def array_to_string(array: c_byte) -> str:
    value = cast(array, c_char_p).value
    if value is None:
        raise ValueError(f"Failed to cast array to string: {array}")
    return value.decode().strip("\0")
