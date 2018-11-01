import cv2
import numpy as np

class Calibration:
    def __init__(self, img, hsv):
        self.img = img
        self.hsv = hsv
        self.calibrations = []
        self.mask = None

    def calibrate(self):
        calibOK = False
        while (not calibOK):
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

            lower, upper = self.getHueRange()
            lower_yellow = np.array(lower)
            upper_yellow = np.array(upper)

            self.mask = cv2.inRange(self.hsv, lower_yellow, upper_yellow)

            cv2.imshow('maskOK?', c.mask)
            key = cv2.waitKey(0)
            print(key)
            print(ord(' '))
            calibOK = ord(' ') == key
            cv2.destroyAllWindows()
        
    
    def clickCallback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.count += 1
            self.calibrations.append(self.img[y, x])
            print("Click recorded for {}, {}".format(x, y))
            print(self.calibrations[-1])

    def getHueRange(self):
        MARGIN_OF_ERROR = 55
        if len(self.calibrations) > 0:
            minB = max(0, min(self.calibrations, key=lambda x : x[0])[0] - MARGIN_OF_ERROR)
            minG = max(0, min(self.calibrations, key=lambda x : x[1])[1] - MARGIN_OF_ERROR)
            minR = max(0, min(self.calibrations, key=lambda x : x[2])[2] - MARGIN_OF_ERROR)
            maxB = min(255, max(self.calibrations, key=lambda x : x[0])[0] + MARGIN_OF_ERROR)
            maxG = min(255, max(self.calibrations, key=lambda x : x[1])[1] + MARGIN_OF_ERROR)
            maxR = min(255, max(self.calibrations, key=lambda x : x[2])[2] + MARGIN_OF_ERROR)
            lower = [minB, minG, minR]
            upper = [maxB, maxG, maxR]
            # print(lower)
            # print(upper)
            # hsvLower = cv2.cvtColor(np.uint8([[lower]]), cv2.COLOR_BGR2HSV)
            # hsvUpper = cv2.cvtColor(np.uint8([[upper]]), cv2.COLOR_BGR2HSV)
            # print(hsvLower)
            # print(hsvUpper)
            return lower, upper
        return [0, 125, 150], [255, 255, 255]




if __name__ == "__main__":
    IMG_PATH = "frame0000.jpg"
    # IMG_PATH = "../assets/imgs/tennis.png"

    img = cv2.imread(IMG_PATH, cv2.IMREAD_COLOR)

    newImageOK = False
    contrast = 0
    brightness = 0
    dx = 2
    while not newImageOK:
        contrast = min(127, contrast)
        brightness = min(127, brightness)
        newImage = np.copy(img) * (contrast / 127 + 1) - contrast + brightness
        newImage = np.clip(newImage, 0, 255)
        newImage = np.uint8(newImage)
        cv2.imshow("brightnessANDcontrast", newImage)
        key = cv2.waitKey(0)
        if key == ord('8'):
            contrast += dx
            print("Adding to contrast: {}".format(contrast))
        elif key == ord('2'):
            contrast -= dx
            print("Removing from contrast: {}".format(contrast))
        elif key == ord('6'):
            brightness += dx
            print("Adding to brightness: {}".format(brightness))
        elif key == ord('4'):
            brightness -= dx
            print("Removing from brightness: {}".format(brightness))
        elif key == ord(' '):
            print("DONE")
            img = newImage
            newImageOK = True
        cv2.destroyAllWindows()

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    print("Calibrating")

    c = Calibration(img, hsv)

    c.calibrate()


    res = cv2.bitwise_and(img, img, mask=c.mask)
    blurred = cv2.medianBlur(res,5)
    print(blurred.shape)
    circles = (cv2.HoughCircles(blurred[:,:,2], cv2.HOUGH_GRADIENT, 1, 50,
               param1=50, param2=50, minRadius=0, maxRadius=0))
    print(circles)
    circles = np.uint16(np.around(circles))
    for i in circles[0,:]:
        # draw the outer circle
        cv2.circle(img,(i[0],i[1]),i[2],(0,255,0),2)
        # draw the center of the circle
        cv2.circle(img,(i[0],i[1]),2,(0,0,255),3)

    cv2.imshow('frame', img)
    cv2.imshow('mask', c.mask)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

