from dataclasses import dataclass
import os
import tqdm
import copy
import rospy
import rosbag
from datetime import datetime

@dataclass
class Options:
    path: str
    output_dir: str
    all = True
    topic_names = ""
    start_time = 0.0
    end_time = 0.0
    no_header = True
    output_type = "json"


def to_datestr(timestamp):
    return datetime.fromtimestamp(timestamp).strftime("%Y/%m/%d %H:%M:%S.%f")


def msg_to_dict(msg):
    msg_dict = {}
    
    for key, value in iter_msg(msg):
        assign_key(msg_dict, key, value)
    
    return msg_dict

def assign_key(tree, key, value, index=0):
    subfield = key[index]
    
    if len(key) == index + 1:
        tree[subfield] = value
        return
    
    if subfield not in tree:
        tree[subfield] = {}

    assign_key(tree[subfield], key, value, index + 1)

def is_list(obj):
    return type(obj) == list or type(obj) == tuple

def iter_msg(msg, key=None):
    msg_type = type(msg)
    if key is None:
        key = []
    key = copy.copy(key)
    if hasattr(msg_type, "__slots__"):
        key.append(None)
        for slot in msg_type.__slots__:
            key[-1] = slot
            val = getattr(msg, slot)
            for result in iter_msg(val, key):
                yield result

    elif is_list(msg):
        key.append(None)
        for index, attr in enumerate(msg):
            key[-1] = index
            for result in iter_msg(attr, key):
                yield result

    else:
        if type(msg) == bytes:
            msg = msg.decode()
        yield key, msg

def get_output_path(options, topic_name=None):
    bag_path = options.path
    bag_filename = os.path.basename(bag_path)
    bag_name = os.path.splitext(bag_filename)[0]
    
    if topic_name is None:
        output_filename = bag_name
    else:
        topic_name = topic_name.replace("/", "-")
        output_filename = "%s-%s" % (bag_name, topic_name)
    
    return os.path.join(options.output_dir, output_filename)

def get_topic_list(path):
    bag = rosbag.Bag(path)
    
    # index 0 is msg type mapped to msg hash
    # index 1 is topic name mapped to TopicTuple (contains msg type, message count, etc)
    topic_tuples = bag.get_type_and_topic_info()[1]  # type: dict[TopicTuple]
    topics = list(topic_tuples.keys())
    return topics


def enumerate_bag(options):

    try:
        num_messages = get_bag_length(options)
        bag = open_bag(options)
        bag_iter = get_bag_iter(bag, options)
        for _ in tqdm.trange(num_messages):
            topic, msg, time = next(bag_iter)

            yield topic, msg, time
    finally:
        bag.close()

def open_bag(options):
    try:
        bag = rosbag.Bag(options.path)
    except Exception as e:
        rospy.logfatal('failed to load bag file: %s', e)
        exit(1)
    return bag

def get_bag_iter(bag, options):
    bag_start = rospy.Time(bag.get_start_time())
    if options.start_time:
        stime = bag_start + rospy.Duration(options.start_time)
    else:
        stime = rospy.Time(bag.get_start_time())
    if options.end_time:
        etime = bag_start + rospy.Duration(options.end_time)
    else:
        etime = rospy.Time(bag.get_end_time())
    return bag.read_messages(topics=options.topic_names, start_time=stime, end_time=etime)

def get_bag_length(options):
    bag = open_bag(options)
    bag_iter = get_bag_iter(bag, options)
    num_messages = sum(1 for _ in bag_iter)
    bag.close()
    return num_messages
