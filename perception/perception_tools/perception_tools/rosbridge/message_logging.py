from perception_tools.rosbridge.types import RawRosMessage


def get_shortened_message(msg: RawRosMessage, max_len: int = 200) -> str:
    msg_str = str(msg)
    if len(msg_str) > max_len:
        return f"{msg_str[:max_len]}..."
    else:
        return msg_str
