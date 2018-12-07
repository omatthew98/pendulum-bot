#!/usr/bin/python
import rospy
from baxter_imaging.msg import BallLoc
import tf
import numpy as np
# from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import PointStamped

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

    # def compute_transform(self, pose, invert = False, q_provided = None, trans_provided = None):
    #     if q_provided is not None:
    #         q = np.array(q_provided)
    #     else:
    #         q = np.array([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
    #     if trans_provided is not None:
    #         translation = np.array(trans_provided)
    #     else:
    #         translation = np.array([pose.position.x, pose.position.y, pose.position.z, 1])
    #     transform = np.array(tf.transformations.quaternion_matrix(q))
    #     if invert:
    #         q = tf.transformations.quaternion_inverse(q)
    #         transform = np.array(tf.transformations.quaternion_matrix(q))
    #         translation = np.dot(transform, translation) * -1
        
    #     transform = np.array(tf.transformations.quaternion_matrix(q))
    #     translation = translation.reshape(4, 1)

    #     transform[0][3] = translation[0]
    #     transform[1][3] = translation[1]
    #     transform[2][3] = translation[2]
    #     return transform

    # def ARCallback(self, markers):
    #     markers = markers.markers # get the markers in the msg
    #     if self.transformation is not None: # transformation is already set do nothing
    #         return
    #     print(self.poses)
    #     if self.cam1_frame_id in self.poses and self.cam2_frame_id in self.poses: # both poses are set so we can calculate the transformation
    #         self.transformation = []
    #         pose1 = self.poses[self.cam1_frame_id]
    #         transform1 = self.compute_transform(pose1)

    #         pose2 = self.poses[self.cam2_frame_id]
    #         transform2 = self.compute_transform(pose2, invert = True)

    #         self.transformation = transform2.dot(transform1)

    #         self.ar_sub.unregister() # don't subscribe to the ar_pose_marker anymore
    #     for marker in markers:
    #         if marker.id == self.ar_tag: # we only want to look at messages that reference the ar_tag we want
    #             if (marker.header.frame_id == self.cam1_frame_id and self.cam1_frame_id not in self.poses): # this is for processing for cam1
    #                 self.poses[marker.header.frame_id] = marker.pose.pose  
    #             elif (marker.header.frame_id == self.cam2_frame_id and self.cam2_frame_id not in self.poses): # this is for processing for c2
    #                 self.poses[marker.header.frame_id] = marker.pose.pose  
    #             # else:
    #             #     pass
        
    # def BallLocCallback(self, bl):
    #     # assert len(self.transforms) > 1, "You must have a starting and ending frame"
    #     if self.transformation is None: # if we don't have a transformation do nothing
    #         return
    #     ballLoc = np.array([bl.x, bl.y, bl.d, 1]).reshape(4, 1) # p_K
    #     if any(map(np.isnan, ballLoc)):
    #         return
    #     ballLoc = self.transformation.dot(ballLoc).reshape(4,) # p_H
    #     # trans, rot = self.listener.waitForTransform("/reference/base", self.cam1_frame_id, rospy.Time(), rospy.Duration(4.0))
    #     # trans.append(1)
    #     # transformation = self.compute_transform(self, None, q_provided = rot, trans_provided = trans)
    #     # ballLoc = np.dot(transformation, ballLoc) # p_B
    #     self.pub.publish(BallLoc(ballLoc[0], ballLoc[1], ballLoc[2], bl.t))
    
def main():
    rospy.init_node('world_frame_transformer')
    wft = WorldFrameTransformer("/kinect/ball/location", "/world/ball/location", "/base")
    rospy.spin()

if __name__ == "__main__":
    main()