#!/usr/bin/python
import rospy
import tf
import baxter_interface
import sensor_msgs.point_cloud2 as pc2

from baxter_imaging.msg import Circ
from sensor_msgs.msg import Image, PointCloud2
import cv_bridge

MARGIN_OF_ERROR = 50.0
TB_DIAMETER = 66.5 # millimeters

class Baxter:
    def __init__(self):
        self.circleSub = rospy.Subscriber('circ_pos', Circ, self.circCallback)
        self.depthSub = rospy.Subscriber('/camera/depth/points', PointCloud2, self.depthCallback)
        self.lastDepth = None
        self.ballPos = None
        self.count = 0
        self.ballPos = None
        self.bridge = cv_bridge.CvBridge()
        self.listener = tf.TransformListener()
    
    def circCallback(self, circle):
        cx, cy = map(int, [circle.x, circle.y])
        # if self.ballPos and ((abs(cx - self.ballPos[0]) > MARGIN_OF_ERROR) or (abs(cy - self.ballPos[1]) > MARGIN_OF_ERROR) or (abs(self.getDepth(cx, cy) - self.ballPos[2]) > MARGIN_OF_ERROR)):
        #     return
        if self.lastDepth is None:
            return
        try:
            nx, ny, nz = self.getDepth(cx, cy)
            if self.ballPos is None:
                self.ballPos = (nx, ny, nz)
            else:
                x, y, z = self.ballPos
                x = (x * self.count + nx) / (self.count + 1)
                y = (y * self.count + ny) / (self.count + 1)
                z = (z * self.count + nz) / (self.count + 1)
                self.ballPos = (x, y, z)
                self.count += 1
            self.grabBall()
        except Exception as e:
            print(e)
        
    def depthCallback(self, pc):
        self.lastDepth = ([x for x in pc2.read_points(pc, skip_nans=True, field_names=("x", "y", "z"))], pc.width)
        
    def getDepth(self, x, y):
        offset = x + y * self.lastDepth[1]
        return self.lastDepth[0][offset]

    def grabBall(self):
        # trans, rot = self.listener.lookupTransform("actual_display", "camera_link", rospy.Time())
        print(self.ballPos)

def convertToMeters(ballPos):
    x, y, z, r = ballPos
    pixelToMeter = TB_DIAMETER / (2 * r)
    return map(lambda x: pixelToMeter * x, (x, y, z))

def main():
    rospy.init_node('grab_ball')
    baxter = Baxter()
    rospy.spin()

if __name__=='__main__':
    main()

