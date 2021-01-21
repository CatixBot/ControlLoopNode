import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError

import SimpleObjectDetector as sod

windowName = "Pick&Find"
pickerKey = ord('r')
exitKey = ord('q')

def drawTrackingRect(imageBGR, features):
    contourColor = (0,255,0)
    rectColor = (255, 255, 255)
    cv2.drawContours(imageBGR, [features.detectedContour], -1, contourColor, 3)
    cv2.circle(imageBGR, features.detectedCenterPoint, 10, rectColor, -1, cv2.FILLED)
    cv2.circle(imageBGR, features.frameCenterPoint, 10, rectColor, -1, cv2.FILLED)
    cv2.rectangle(imageBGR, features.detectedCenterPoint, features.frameCenterPoint, rectColor, 3)

def drawAnnotations(imageBGR, features):
    fontFace = cv2.FONT_HERSHEY_SIMPLEX
    fontScale = 1
    fontThickness = 1
    fontColor = (255, 255, 255)
    textIndent = int(10) 
    drawnText = "delta Y: {}".format(features.axialDelta[1])
    textCoord = (textIndent, int(imageBGR.shape[0] - textIndent))
    cv2.putText(imageBGR, drawnText, textCoord, fontFace, fontScale, fontColor, fontThickness)
    drawnText = "delta X: {}".format(features.axialDelta[0])
    textBoundingRect, _ = cv2.getTextSize(drawnText, fontFace, fontScale, fontThickness)
    textCoord = (textIndent, int(imageBGR.shape[0] - 2*textIndent - textBoundingRect[1]))
    cv2.putText(imageBGR, drawnText, textCoord, fontFace, fontScale, fontColor, fontThickness)

def drawDetected(imageBGR, features):
    drawTrackingRect(imageBGR, features)
    drawAnnotations(imageBGR, features)

def waitKey():
    return cv2.waitKey(5) & 0xFF

def showImage(imageBGR):
    cv2.imshow(windowName, imageBGR)
    cv2.waitKey(3)

class SimpleControlLoop:
    def __init__(self):
        self.brige = CvBridge()
        self.cameraSubscriber = rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, self.callback)
        self.rangeTracker = sod.RangeDetectorHSV([])
        self.colorRangesHSV = []

    def callback(self, data):
        try:
            cameraImage = self.brige.compressed_imgmsg_to_cv2(data, "passthrough")
        except CvBridgeError as e:
            print(e)
        (rows, cols, channels) = cameraImage.shape
        if cols == 0 or rows == 0:
            return
        keyPressed = waitKey()
        if keyPressed == pickerKey:
            rangePicker = sod.RangePickerHSV(windowName)
            colorRange = rangePicker.pickRange(cameraImage)
            self.colorRangesHSV.append((colorRange.lowerBound, colorRange.upperBound))
            self.rangeTracker = sod.RangeDetectorHSV(self.colorRangesHSV)
        else:
            detectedFeatures = self.rangeTracker.detectRange(cameraImage)
            for features in detectedFeatures:
                drawDetected(cameraImage, features)
            showImage(cameraImage)

def main(args):
    controlLoop = SimpleControlLoop()
    rospy.init_node("catix_control_loop", anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("shutting down")
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main(sys.argv)
