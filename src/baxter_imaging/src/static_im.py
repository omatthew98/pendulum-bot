import numpy as np
import cv2

img = cv2.imread("tennis.png", cv2.IMREAD_COLOR)
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

# Thresholds for extracting the tennis ball
# lower = [int(x) for x in input("Enter three lower numbers:").split(" ")]
# upper = [int(x) for x in input("Enter three upper numbers:").split(" ")]
lower = [0, 125, 150]
upper = [255, 255, 255]
lower_yellow = np.array(lower)
upper_yellow = np.array(upper)

mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

# Bitwise-AND mask and original image
res = cv2.bitwise_and(img, img, mask=mask)
blurred = cv2.medianBlur(res,5)
print(blurred.shape)
circles = (cv2.HoughCircles(blurred[:,:,2], cv2.HOUGH_GRADIENT, 1, 50,
           param1=50, param2=50, minRadius=0, maxRadius=0))
    
circles = np.uint16(np.around(circles))
for i in circles[0,:]:
    # draw the outer circle
    cv2.circle(img,(i[0],i[1]),i[2],(0,255,0),2)
    # draw the center of the circle
    cv2.circle(img,(i[0],i[1]),2,(0,0,255),3)

cv2.imshow('frame', img)
cv2.imshow('mask', mask)
cv2.imshow('res', res)
cv2.waitKey(0)
cv2.destroyAllWindows()
