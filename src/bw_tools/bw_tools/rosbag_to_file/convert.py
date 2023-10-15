#!/usr/bin/python3
# -*- coding: utf-8 -*-
import sys
import rospy

from optparse import OptionParser

from bw_tools.rosbag_to_file import utils

from bw_tools.rosbag_to_file.rosbag_to_csv import bag_to_csv
from bw_tools.rosbag_to_file.rosbag_to_json import bag_to_json

ALLOWED_TYPES = ["csv", "json"]


def main(options):
    if len(options.path) == 0:
        rospy.logerr("Please provide a bag path with --path")
        sys.exit(1)
    
    if len(options.output_dir) == 0:
        rospy.logerr("Please provide an output path with --output_dir")
        sys.exit(1)

    topics = utils.get_topic_list(options.path)
    if options.all_topics:
        options.topic_names = topics

    if len(options.topic_names) == 0:
        rospy.logerr("Please select topics")
        sys.exit()

    rospy.loginfo("Converting to %s" % options.output_type)
    if options.output_type == "csv":
        bag_to_csv(options)
    elif options.output_type == "json":
        bag_to_json(options)
    else:
        message = "Invalid output type: %s. Allowed types are: %s" % (options.output, ", ".join(ALLOWED_TYPES))
        rospy.logerr(message)
        sys.exit()


def parse_opts():
    parser = OptionParser(usage="%prog [options] bagfile")
    parser.add_option("-a", "--all", dest="all_topics",
            action="store_true",
            help="exports all topics", default=True)
    parser.add_option("-t", "--topic", dest="topic_names",
            action="append", help="white list topic names", metavar="TOPIC_NAME")
    parser.add_option("-s", "--start-time", dest="start_time",
                      help="start time of bagfile", type="float")
    parser.add_option("-e", "--end-time", dest="end_time",
                      help="end time of bagfile", type="float")
    parser.add_option("-n", "--no-header", dest="header",
                      action="store_false", default=True,
                      help="no header / flatten array value")
    parser.add_option("-p", "--path", dest="path", default="",
                      help="bag file to parse")
    parser.add_option("--type", dest="output_type", choices=ALLOWED_TYPES,
                      default="json", help="output file type")
    parser.add_option("-o", "--output_dir", dest="output_dir", default=".",
                      help="output dir")
    options, args = parser.parse_args()

    return options

if __name__ == '__main__':
    options = parse_opts()
    main(options)
