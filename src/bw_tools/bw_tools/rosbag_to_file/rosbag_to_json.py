import json

from . import utils

def bag_to_json(options):
    stream_array = StreamArray(json_generator(options))
    path = utils.get_output_path(options)
    path += ".json"
    with open(path, 'w') as outfile:
        for chunk in json.JSONEncoder(indent=4).iterencode(stream_array):
            outfile.write(chunk)


def json_generator(options):
    for topic, msg, timestamp in utils.enumerate_bag(options):
        msg_dict = utils.msg_to_dict(msg)

        yield [
            timestamp.to_time(),
            topic,
            msg_dict,
        ]

class StreamArray(list):
    """
    Converts a generator into a list object that can be json serialisable
    while still retaining the iterative nature of a generator.

    IE. It converts it to a list without having to exhaust the generator
    and keep it's contents in memory.
    """
    def __init__(self, generator):
        self.generator = generator
        self._len = 1

    def __iter__(self):
        self._len = 0
        for item in self.generator:
            yield item
            self._len += 1

    def __len__(self):
        """
        Json parser looks for a this method to confirm whether or not it can
        be parsed
        """
        return self._len
