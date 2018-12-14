#!/usr/bin/python
import rospy
from baxter_imaging.msg import BallLoc
import tf
import numpy as np
from geometry_msgs.msg import PointStamped

LOCATION_TOPIC_KINECT="~location_topic/kinect"
LOCATION_TOPIC_BASE="~location_topic/base"
TARGET_FRAME="~frame/target"

class WorldFrameTransformer:
    def __init__(self, sub_topic, pub_topic, target_frame):
        self.listener = tf.TransformListener()
        self.trans = tf.TransformerROS()
        self.target_frame = target_frame
        self.ball_loc = rospy.Subscriber(sub_topic, PointStamped, self.pointCB)
        self.pub = rospy.Publisher(pub_topic, PointStamped)

    def pointCB(self, point):
        try:
            t, r = self.listener.lookupTransform(self.target_frame, point.header.frame_id, rospy.Time())
        except:
            return
        transformation = self.trans.fromTranslationRotation(t, r)
        p = np.array([point.point.x, point.point.y, point.point.z, 1]).reshape(4,1)
        ps = PointStamped()
        ps.point.x, ps.point.y, ps.point.z, _ = transformation.dot(p) .reshape(4,)
        ps.header.frame_id = self.target_frame
        self.pub.publish(ps)

def main():
    rospy.init_node('world_frame_transformer')
    if not rospy.has_param(LOCATION_TOPIC_KINECT):
        rospy.logerr("Couldn't find parameter %s", LOCATION_TOPIC_KINECT)
        exit(1)
    if not rospy.has_param(LOCATION_TOPIC_BASE):
        rospy.logerr("Couldn't find parameter %s", LOCATION_TOPIC_BASE)
        exit(1)
    if not rospy.has_param(TARGET_FRAME):
        rospy.logerr("Couldn't find parameter %s", TARGET_FRAME)
        exit(1)
    wft = WorldFrameTransformer(
        rospy.get_param(LOCATION_TOPIC_KINECT), 
        rospy.get_param(LOCATION_TOPIC_BASE), 
        rospy.get_param(TARGET_FRAME))
    rospy.spin()

if __name__ == "__main__":
    main()