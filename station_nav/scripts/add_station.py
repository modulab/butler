#!/usr/bin/python

import rospy

import sys
import os
from optparse import OptionParser
from geometry_msgs.msg import PoseWithCovarianceStamped
import yaml

if __name__ == "__main__":
    rospy.init_node("station_saver")
    parser = OptionParser()
    parser.add_option("-f", "--file", dest="yaml_filename",
                      help="write or add station to FILE", metavar="FILE")
    parser.add_option("-i", "--identity",
                      dest="id",
                      help="station id (integer)")
    parser.add_option("-o", "--overwrite", action="store_true",
                      dest="overwrite", default=False,
                      help="overwrite existing station if already exists")
    (options, args) = parser.parse_args()


    rospy.loginfo("Storing a new station in file '%s'"%options.yaml_filename)

    if options.yaml_filename is None:
        rospy.logerr("Need a file name")
        sys.exit(1)

    if os.path.isfile(options.yaml_filename):
        rospy.loginfo("Adding to existing file")
        with open(options.yaml_filename,"r") as f:
            yml = yaml.load(f)
    else:
        yml={}

    if options.id is None:
        ks = [int(k) for k in yml.keys()]
        if len(ks) < 1:
            id = 0
        else:
            id = max(ks) + 1
    else:
        id = int(options.id)

    rospy.loginfo("Station id: %d",id)
    if str(id) in yml.keys() and not options.overwrite:
        rospy.logerr("Station with id %d already exists, and overwrite false."%id)
        sys.exit(1)

    rospy.loginfo("Getting AMCL pose")
    try:
        pose = rospy.wait_for_message("/amcl_pose", PoseWithCovarianceStamped, 10)
    except:
        rospy.logerr("Can't get pose!")
        sys.exit(1)

    yml[str(id)]=pose.pose.pose

    rospy.loginfo("Saving yaml.")
    with open(options.yaml_filename, "w") as f:
        f.write(yaml.dump(yml))
    rospy.loginfo("Done")

