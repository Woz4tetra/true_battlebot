import csv

from . import utils

def bag_to_csv(options):
    writers = dict()
    for topic, msg, timestamp in utils.enumerate_bag(options, options.path):
        if topic in writers:
            writer = writers[topic][0]
        else:
            f = open_csv(options, topic)
            writer = csv.writer(f)
            writers[topic] = writer, f
            # header
            if options.header:
                header_row = ["date", "time"]
                message_type_to_csv(header_row, msg)
                writer.writerow(header_row)

        row = [
            utils.to_datestr(timestamp.to_time()),
            timestamp.to_time()
        ]
        message_to_csv(row, msg, flatten=not options.header)
        writer.writerow(row)
    for writer, f in writers.values():
        f.close()


def open_csv(options, topic_name):
    path = utils.get_output_path(options, topic_name)
    path += ".csv"
    return open(path, 'w')


def message_to_csv(row, msg, flatten=False):
    """
    row: list
    msg: message
    """
    for key, value in utils.iter_msg(msg):
        msg_str = str(value)
        if msg_str.find(",") != -1:
            if flatten:
                msg_str = msg_str.strip("(")
                msg_str = msg_str.strip(")")
                msg_str = msg_str.strip(" ")
            else:
                msg_str = "\"" + msg_str + "\""
        row.append(msg_str)


def format_header_key(key):
    header = ""
    for index, subfield in enumerate(key):
        if type(subfield) == int:
            header += "[%s]" % subfield
        else:
            if index == 0:
                header += subfield
            else:
                header += "." + subfield
    return header

def message_type_to_csv(row, msg, parent_content_name=""):
    """
    row: list
    msg: message
    """
    for key, value in utils.iter_msg(msg):
        row.append(format_header_key(key))
