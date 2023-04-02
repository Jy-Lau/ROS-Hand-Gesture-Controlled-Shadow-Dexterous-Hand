#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from cvzone.HandTrackingModule import HandDetector
from std_msgs.msg import String, Int32MultiArray
import yaml
import rospkg

class Camera():

    def __init__(self):
        rospy.init_node('camera_node', anonymous=True)
        rospy.loginfo("Camera...")
        self.plan_publisher = rospy.Publisher('/plan', Int32MultiArray, queue_size=10)
        hand_grasps_path = rospkg.RosPack().get_path('hand_robot') + f"/config/numbers.yaml"
        self.config={}
        self._open_yaml(hand_grasps_path)
        
    def _open_yaml(self, path):
        with open(path, 'r') as f:
            self.config = yaml.safe_load(f)

    def run(self):
        cv2.namedWindow('Frame')

        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            rospy.logerr("Camera index 0 is not available!")
            exit()
        detector = HandDetector(detectionCon=0.8, maxHands=1)
        while not rospy.is_shutdown():
            # Get image frame
            success, img = cap.read()
            # Find the hand and its landmarks
            hands, img = detector.findHands(img)  # with draw
            if len(hands)==1 and hands[0]["type"] == "Right":
                # Hand 1
                hand1 = hands[0]
                lmList1 = hand1["lmList"]  # List of 21 Landmark points
                bbox1 = hand1["bbox"]  # Bounding box info x,y,w,h
                centerPoint1 = hand1['center']  # center of the hand cx,cy
                handType1 = hand1["type"]  # Handtype Left or Right

                fingers1 = detector.fingersUp(hand1)
                self.plan_publisher.publish(Int32MultiArray(data=fingers1))
                rospy.loginfo(f"Gesture: {fingers1}")
                # length, info, img = detector.findDistance(lmList1[0][0:2], lmList1[13][0:2], img)  # with draw
                cv2.rectangle(img, (bbox1[0] - 20, bbox1[1] - 20),
                                  (bbox1[0] + bbox1[2] + 20, bbox1[1] + bbox1[3] + 20),
                                  (255, 0, 255), 2)
                for key, value in self.config.items():
                    for number in value:
                        if number == fingers1:
                            cv2.putText(img, key, (bbox1[0] + bbox1[2] -40, bbox1[1] - 30), cv2.FONT_HERSHEY_SIMPLEX,
                                    1, (0, 255, 0), 2)
            cv2.imshow('Frame', img)
            k = cv2.waitKey(1) & 0xFF
            if k == 27:
                break
        cap.release()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    camera = Camera()
    try:
        camera.run()
    except rospy.ROSInterruptException:
        pass