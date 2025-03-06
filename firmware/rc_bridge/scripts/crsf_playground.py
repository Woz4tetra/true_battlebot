from bw_shared.radio.crsf.crsf_parser import CrsfParser


def input_data_to_bytes(data: str) -> bytes:
    result = b""
    for encoded_byte in data.split(" "):
        result += int(encoded_byte, 16).to_bytes(1, "big")
    return result


def parse(data: bytes) -> list:
    results = []
    while len(data) > 0:
        if data[0] != 0xC8:
            data = data[1:]
            continue
        frame_length = data[1]
        if not (2 <= frame_length <= 64):
            data = data[1:]
            continue
    return results


def main() -> None:
    # data = "C8 64 E1 A0 B4 E2 C7 21 15 13 7C 3 84 C0 90 82 81 88 3D"
    data = "C8 88 24 2 64 2 2 41 12 1 58"
    data_decoded = input_data_to_bytes(data)
    results = parse(data_decoded)
    if len(results) == 0:
        print("No results")


if __name__ == "__main__":
    main()
