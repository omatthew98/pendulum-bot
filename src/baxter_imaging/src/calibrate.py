import cv2
import numpy as np

class Calibration:
    def __init__(self, img, hsv):
        self.img = img
        self.hsv = hsv
        self.count = 0
        self.calibrations = []

    def calibrate(self):
        self.count = 0
        self.calibrations = []
        cv2.namedWindow('calibration')
        cv2.setMouseCallback('calibration', self.clickCallback)
        while self.count < 10:
            cv2.imshow('calibration', self.img)
            key = cv2.waitKey(1) & 0xFF

            if key == ord('\n'):
                break

        cv2.destroyAllWindows()
    
    def clickCallback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.count += 1
            self.calibrations.append(self.img[y, x])
            print("Click recorded for {}, {}".format(x, y))
            print(self.calibrations[-1])

    def getHueRange(self):
        if len(self.calibrations) > 0:
            minB = img[min(self.calibrations, key=lambda x : x[2])][2]
            minG = img[min(self.calibrations, key=lambda x : x[1])][1]
            minR = img[min(self.calibrations, key=lambda x : x[0])][0]
            maxB = img[max(self.calibrations, key=lambda x : x[2])][2]
            maxG = img[max(self.calibrations, key=lambda x : x[1])][0]
            maxR = img[max(self.calibrations, key=lambda x : x[0])][1]
            print(minB)
            lower = [minR, minG, minB]
            upper = [maxR, maxG, maxB]
            print(lower)
            print(upper)
            hsvLower = cv2.cvtColor(np.uint8([[lower]]), cv2.COLOR_BGR2HSV)
            hsvUpper = cv2.cvtColor(np.uint8([[upper]]), cv2.COLOR_BGR2HSV)
            print(hsvLower)
            print(hsvUpper)
            return hsvLower[0][0], hsvUpper[0][0]
        return [0, 125, 150], [255, 255, 255]


# IMG_PATH = "../assets/imgs/baxter_tennis_ball.png"
IMG_PATH = "../assets/imgs/tennis.png"

img = cv2.imread(IMG_PATH, cv2.IMREAD_COLOR)
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

print("Calibrating")

c = Calibration(img, hsv)
c.calibrate()

lower, upper = c.getHueRange()
lower_yellow = np.array(lower)
upper_yellow = np.array(upper)

mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

cv2.imshow('frame', img)
cv2.imshow('mask', mask)
cv2.waitKey(0)
cv2.destroyAllWindows()

