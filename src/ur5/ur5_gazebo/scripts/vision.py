#! /usr/bin/env python3

# --------------------- IMPORT ---------------------- #

import cv2
import math
import numpy as np
import torch
import message_filters
import copy
import rospy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import String



# ---------------- GLOBAL VARIABLES ------------------ #

table_origin = (0.7, 0)
table_size   = (0.913, 0.913)
cam_origin  = (0.7, 0, 1.580)



# ----------------- UTILITY FUNCTIONS ----------------- #

def lego_height(lego_class):                                                                            # to obtain lego height
    if lego_class in [6, 7]:
        return 0.057/2
    elif lego_class in [1, 2, 3, 4, 5, 8, 10]:
        return 0.028
    elif lego_class in [0, 9]:
        return 0.019
    else:
        raise ValueError("Invalid class id: {}".format(lego_class))


def discriminate(depth_map):                                                                            # to discriminate between class 5 and 9 due to similar geometry
    delta = depth_map.max() - depth_map.min()                                                           # taking the difference between the table surface and the highest poin of the lego
    if delta < 0.050:
        return 9, "lego_X1-Y4-Z1"                                                                       # if we find that delta to be less than 0.050 (empiric value) we return 9 as the class of the lego
    else:
        return 5, "lego_X1-Y4-Z2"                                                                       # if we find that delta to be more than 0.050 (empiric value) we return 5 as the class of the lego


# ------------------ MAIN FUNCTION ------------------ #

def camera_callback(rgb_img, depth_map):
    rgb_img = CvBridge().imgmsg_to_cv2(rgb_img, "bgr8")                                                 # conversion of the table disposal from ROS to a standard image
    depth_map = CvBridge().imgmsg_to_cv2(depth_map, "32FC1")                                            # conversion of the black and white depth map from ROS to a standard image

    img_clone = copy.deepcopy(rgb_img).astype(np.uint8)                                                 # clonig the original take to work on a backup


    with torch.no_grad():                                                                               # disabling gradient
        bounding_boxes = yolo(rgb_img[:, :, ::-1]).pandas()                                             # ::-1 invert color canal from BGR to RGB

    message = ""                                                                                        # message resetting for next data sending
    for x1, y1, x2, y2, yolo_conf, lego_class, lego_name in bounding_boxes.xyxy[0].to_numpy()[:, :7]:
        if yolo_conf < 0.7:                                                                             # checking yolo confidence to be at least 70%
            continue                                                                                    # skipping the cycle if we find confidence to be too low

        x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
        crop_depth = depth_map[y1:y2, x1:x2]

        if lego_class == 5 or lego_class == 9:                                                          # checking the class id to be 5 or 9
            lego_class, lego_name = discriminate(crop_depth)                                            # using the utility function to discriminate between lego id 5 and lego id 9


        bbox_center = ((x1 + x2) // 2, (y1 + y2) // 2)                                                  # get center of bounding box


        crop_contour = np.where(crop_depth < crop_depth.max() - 0.005, 255, 0).astype(np.uint8)         # crop the bounding box
        crop_contour, _ = cv2.findContours(crop_contour, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)      #
        try:                                                                                            #
            crop_contour = crop_contour[np.argmax([len(x) for x in crop_contour], axis=0)]              #
        except ValueError:                                                                              #
            continue                                                                                    #

        lego_siluette = cv2.minAreaRect(crop_contour)                                                   # obtaining the lego siluette (more precise than the bounding box)

        lego_coord = np.array([int(lego_siluette[0][0] + x1), int(lego_siluette[0][1] + y1)], dtype=np.float32) # creating a num py array with x and y
        lego_coord = lego_coord / (rgb_img.shape[1], rgb_img.shape[0])
        lego_coord[0] = 1 - lego_coord[0]
        lego_coord = lego_coord[::-1]
        lego_coord[0] = 1 - lego_coord[0]
        lego_coord -= 0.5
        lego_coord -= lego_coord*2*0.02                                                                 # correction in order of dont miss data in perspective
        lego_coord *= table_size
        lego_coord += table_origin

        lego_z   = cam_origin[2] - depth_map[bbox_center] + lego_height(lego_class)                     # obtaining lego z coordinate

        box = cv2.boxPoints(lego_siluette) + (x1, y1)
        box = np.int0(box)
        cv2.drawContours(img_clone, [box], 0, (0, 0, 255), 2)

        angle = lego_siluette[2] + 90 if lego_siluette[1][0] < lego_siluette[1][1] else lego_siluette[2]
        angle %= 180
        angle = 180 - angle
        yaw = angle/180 * math.pi
        pitch = 0
        roll = 0

        message += str(lego_class) + " " + str(lego_coord[0]) + " " + str(lego_coord[1]) + " " + str(lego_z)
        message += " " + str(yaw) + " " + str(pitch) + " " + str(roll) + "\n"

        cv2.putText(img_clone, f"{lego_class} {lego_coord[0]:.2f} {lego_coord[1]:.2f} {lego_z:.3f}", (x1-100, y2+10), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 0), 1)
    cv2.imshow("Predictions", img_clone)
    cv2.waitKey(1)

    pub.publish(String(message))


if __name__ == '__main__':
    device = "cpu"

    print("Starting")
    rospy.init_node("yolo")

    yolo = torch.hub.load(
        "src/ur5/ur5_gazebo/scripts/yolov5-master",
        "custom",
        path="src/ur5/ur5_gazebo/scripts/yolov5-master/best.pt",
        source="local").eval()

    pub = rospy.Publisher("yolo", String, queue_size=1)

    sub_rgb_img = message_filters.Subscriber("/camera/color/image_raw", Image)
    sub_depth_map = message_filters.Subscriber("/camera/depth/image_raw", Image)
    ts = message_filters.TimeSynchronizer([sub_rgb_img, sub_depth_map], 10)
    ts.registerCallback(camera_callback)

    print("Waiting for images")
    rospy.spin()
