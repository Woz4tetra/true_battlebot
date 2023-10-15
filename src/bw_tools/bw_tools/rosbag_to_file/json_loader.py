import os
import json
import pickle
from scipy.spatial.transform import Rotation


def iter_bag(path):
    with open(path) as file:
        bag = json.load(file)

    for row in bag:
        yield row


def get_key(tree, key, default=None):
    key = key.split(".")
    try:
        return get_key_recurse(tree, key, 0)
    except KeyError:
        return default


def get_key_recurse(tree, key, index):
    subfield = key[index]
    if index + 1 == len(key):
        return tree[subfield]
    else:
        return get_key_recurse(tree[subfield], key, index + 1)


def yaw_from_quat(quat):
    r_mat = Rotation.from_quat([quat["x"], quat["y"], quat["z"], quat["w"]])
    return r_mat.as_euler("xyz")[2]


def header_to_stamp(header):
    return header["secs"] + header["nsecs"] * 1E-9


def get_pkl_path(path):
    pkl_dir = os.path.dirname(path)
    pkl_name = os.path.splitext(os.path.basename(path))[0]
    pkl_filename = pkl_name + ".pkl"
    pkl_path = os.path.join(pkl_dir, pkl_filename)
    return pkl_path


def make_pkl(path, get_states):
    states = get_states(path)

    pickle_path = get_pkl_path(path)
    with open(pickle_path, 'wb') as file:
        pickle.dump(states, file)

    return states


def read_pkl(path, get_states, repickle=False):
    pkl_path = get_pkl_path(path)
    if not os.path.isfile(pkl_path) or repickle:
        return make_pkl(path, get_states)
    else:
        with open(pkl_path, 'rb') as file:
            return pickle.load(file)
