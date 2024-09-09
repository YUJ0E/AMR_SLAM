#!/usr/bin/env python3
import rosbag
import tf
import sys
from geometry_msgs.msg import TransformStamped

if len(sys.argv) < 3:
    print("Usage: python bag_tf_to_tum.py input_bag.bag output_file.tum")
    sys.exit(1)

input_bag_path = sys.argv[1]
output_file_path = sys.argv[2]

def transform_to_tum_string(transform):
    t = transform.transform.translation
    r = transform.transform.rotation
    return "{0} {1} {2} {3} {4} {5} {6} {7}".format(transform.header.stamp.to_sec() +1710398250.532337 - 140, t.x, t.y, t.z, r.x, r.y, r.z, r.w)

print(input_bag_path, output_file_path)
try:
    with rosbag.Bag(input_bag_path, 'r') as bag, open(output_file_path, 'w') as output_file:
        for topic, msg, t in bag.read_messages(topics=['/tf', '/tf_static']):
            for transform in msg.transforms:
                if transform.header.frame_id == "odom" and transform.child_frame_id == "base_footprint":
                    output_file.write(transform_to_tum_string(transform) + '\n')
except Exception as e:
    print("Failed to process bag file: ", e)
    sys.exit(1)

print("Finished converting tf messages to TUM format.")
