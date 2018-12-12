import rospy

from geometry_msgs.msg import PointStamped



class Calibration:

    def __init__(self, sub_topic, pub_topic):
        self.r = 1.2
        self.sub = rospy.Subscriber(sub_topic, PointStamped, self.callback)
        self.pub = rospy.Publisher(pub_topic, PointStamped)

    def callback(self, calib_point):
        # if len(self.xdata) > self.length:
        #     return
        # self.xc, self.yc, self.zc = data.point.x, data.point.y, (data.point.z + self.r)
        # self.xdata.append(self.xc)
        # self.ydata.append(self.yc)
        # self.zdata.append(self.zc)
        # if len(self.xdata) == 1:
        #     return
        # self.xc = sum(self.xdata) / len(self.xdata)
        # self.yc = sum(self.ydata) / len(self.ydata)
        # self.zc = sum(self.zdata) / len(self.zdata)
        # print("Length of the list", len(self.xdata))
        self.xc, self.yc, self.zc, self.r = calib_point.point.x, calib_point.point.y, calib_point.point.z + self.r
        result = PointStamped()
        result.point.x = self.xc
        result.point.y = self.yc
        result.point.z = self.zc

        self.pub.publish(result)


def main():
    rospy.init_node("calibration")
    calib = Calibration("world/ball/location", "world/ball/calibraion")
    rospy.spin()

if __name__ == "__main__":
    main()