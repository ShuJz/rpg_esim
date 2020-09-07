#!/usr/bin/env python

from __future__ import print_function

import sys
import rospy
import cv2
import numpy as np
import csv
from esim_srvs.srv import EsimSrv, EsimSrvRequest, EsimSrvResponse
from cv_bridge import CvBridge

def esim_client(image_message):
    print('Waitting for server event_smul')
    rospy.wait_for_service('esim_smul')
    try:
        req = EsimSrvRequest()
        req.image = image_message
        esim_simulator = rospy.ServiceProxy('esim_smul', EsimSrv)
        resp1 = esim_simulator(req)
        header = resp1.header
        height = resp1.height
        width = resp1.width
        events = resp1.events
        # print('height {}, width {}'.format(height, width))
        if (len(events) > 0):
            return make_events_frame(events, height, width)
        else:
            return None
    except rospy.ServiceException as e:
        print("Service call failed")


def make_events_frame(events, height, width):
        frame = np.zeros((height, width), dtype=np.uint8)
        for event in events:
            frame[event.y, event.x] = 1
        return frame

if __name__ == "__main__":

    image_folder = sys.argv[1]

    bridge = CvBridge()
    with open(image_folder + '/images.csv') as f:
        f_csv = csv.reader(f)
        headers = next(f_csv)
        while True:
            for row in f_csv:
                time_stamp = rospy.Time(int(row[0]))
                image_name = row[1]
                cv_image = cv2.imread(image_folder + '/' + image_name)
                gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
                image_message = bridge.cv2_to_imgmsg(gray, encoding="8UC1")
                image_message.header.stamp = time_stamp
                image_message.width
                image_message.height
                frame = esim_client(image_message)
                cv2.imshow('input', cv_image)
                cv2.waitKey(1)
                if (frame is not None):
                    cv2.imshow('output', frame * 255)
                    cv2.waitKey(1)
               
