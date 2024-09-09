import rospy
import tf
import csv
from geometry_msgs.msg import TransformStamped

rospy.init_node('tf_listener')
listener = tf.TransformListener()
rate = rospy.Rate(10.0)

with open('carto.tum', 'w') as file:
    writer = csv.writer(file, delimiter=' ')
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
            now = rospy.Time.now()
            writer.writerow([now.to_sec(), trans[0], trans[1], trans[2], rot[0], rot[1], rot[2], rot[3]])
            rate.sleep()
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        except rospy.exceptions.ROSTimeMovedBackwardsException:
            rospy.logwarn("Time moved backwards. Ignoring the time jump and continuing.")
            continue
