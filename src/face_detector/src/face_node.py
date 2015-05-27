#!/usr/bin/python
"""
This program is demonstration for face and object detection using haar-like features.
The program finds faces in a camera image or video stream and displays a red box around them.

Original C implementation by:  ?
Python implementation by: Roman Stanchak, James Bowman
"""
import sys
import cv2.cv as cv
from optparse import OptionParser
import rospy
import numpy
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

min_size = (20, 20)
image_scale = 2
haar_scale = 1.2
min_neighbors = 2
haar_flags = 0


def get_image(data):
    bridge=CvBridge()
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")

    # cascade = cv.Load('/home/turtlebot/Downloads/haarcascade_frontalface_alt.xml')
    cascade = cv2.CascadeClassifier('/usr/local/include/opencv-2.4.8/data/haarcascades/haarcascade_frontalface_alt.xml')

    detect_and_draw(cv_image, cascade)


def detect_and_draw(img, cascade):
    gray = cv.CreateImage((img.shape[1],img.shape[0]), 8, 1)
    small_img = cv.CreateImage((cv.Round(img.shape[1] / image_scale),
                   cv.Round (img.shape[0] / image_scale)), 8, 1)

    img=cv.fromarray(img)
    cv.CvtColor(img, gray, cv.CV_BGR2GRAY)

    cv.Resize(gray, small_img, cv.CV_INTER_LINEAR)

    cv.EqualizeHist(small_img, small_img)

    if(cascade):
        # t = cv.GetTickCount()
        # faces = cv.HaarDetectObjects(small_img, cascade, cv.CreateMemStorage(0),
        #                              haar_scale, min_neighbors, haar_flags, min_size)
        # t = cv.GetTickCount() - t
        # #print "detection time = %gms" % (t/(cv.GetTickFrequency()*1000.))
        # if faces:
        #     for ((x, y, w, h), n) in faces:
        #         # the input to cv.HaarDetectObjects was resized, so scale the
        #         # bounding box of each face and convert it to two CvPoints
        #         pt1 = (int(x * image_scale), int(y * image_scale))
        #         pt2 = (int((x + w) * image_scale), int((y + h) * image_scale))
        #         cv.Rectangle(img, pt1, pt2, cv.RGB(255, 0, 0), 3, 8, 0)
        
        imgInds = tuple( int(((x+0.5*w)+640*(y+0.5*h))*image_scale) for (x, y, w, h) in faces)
        idxPub=rospy.Publisher('face_idxs',Int32MultiArray)
        msg = Int32MultiArray(data=imgInds)
        idxPub.publish(msg)

    ##########################rosnode
    # img_pub=rospy.Publisher('face_in_img',Image,queue_size=10)
    # bridge=CvBridge()
    # msg=bridge.cv2_to_imgmsg(numpy.asarray(img), "bgr8")
    # img_pub.publish(msg)
    #cv.ShowImage("result", img)

if __name__ == '__main__':

    rospy.init_node('face_detector_node',anonymous=True)
    rospy.Subscriber('/camera/rgb/image_raw',Image,get_image)

    parser = OptionParser(usage = "usage: %prog [options] [filename|camera_index]")
    parser.add_option("-c", "--cascade", action="store", dest="cascade", type="str", help="Haar cascade file, default %default", default = "../data/haarcascades/haarcascade_frontalface_alt.xml")
    (options, args) = parser.parse_args()
    
    #cascade = cv.Load('/home/pipikk/Downloads/opencv-2.4.8/data/haarcascades/haarcascade_frontalface_alt.xml')
    # cascade = cv.Load('/home/turtlebot/Downloads/haarcascade_frontalface_alt.xml')
    
    
    cascade = cv2.CascadeClassifier('/usr/local/include/opencv-2.4.8/data/haarcascades/haarcascade_frontalface_alt.xml')

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"
