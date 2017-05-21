__author__ = 'Erol Bahadiroglu'

'''
This script receives images from one of the submarine's cameras (stereo/left) and searches for a shape resembles a
gateway. This involves applying the Harris Corner Detection tool (from OpenCV), constructing a set of vectors between
the corner points, and finding overlapping pairs of perpendicular vectors.

Output is sent with the topic 'gatway'. It is a JSON with the format [[top left corner], [dimensions]].
'''

import cv2
import matplotlib.pyplot as plt
import numpy as np
import rospy
from sensor_msgs.msg import Image as ROS_img
from std_msgs.msg import String as ROS_string
import cv_bridge
import json

camera = 'stereo/left'  # address of camera
destination = 'gateway'  # topic where data is published
#input_image = cv2.imread("images.png")  # testing image

############### IMAGE PROCESSING ###############

class Vector(object):
    def __init__(self, unit_vector, magnitude, point1, point2):
        self.unit = unit_vector
        self.magnitude = magnitude
        self.point_set = (point1, point2)

    def __repr__(self):
        return "unit vector: " + str(self.unit) + "; magnitude: " + str(self.magnitude) + "; " + str(self.point_set)

    def equals(self, other):
        if (self.magnitude == other.magnitude) and (set(self.unit) == set(other.unit)):
            return True
        else:
            return False

    @staticmethod
    def unit_dot(v1, v2):
        return (v1.unit[0] * v2.unit[0]) + (v1.unit[1] * v2.unit[1])

    @staticmethod
    def is_ortho(v1, v2, tolerance):
        product = Vector.unit_dot(v1,v2)
        if abs(product) < tolerance:
            return True
        else:
            return False

    @staticmethod
    def is_parallel(v1, v2, tolerance):
        product = Vector.unit_dot(v1,v2)
        if (1 - abs(product)) < tolerance:
            return True
        else:
            return False

def color_filter(image, target_hue=None, t_hue=None, target_saturation=None, t_sat=None, target_brightness=None, t_brt=None):
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    masks = []
    if target_hue:
        hue_matrix = hsv_image[:,:,0]
        mask = cv2.inRange(hue_matrix, target_hue-t_hue, target_hue+t_hue)
        masks.append(mask)
    if target_saturation:
        saturation_matrix = hsv_image[:,:,1]
        mask = cv2.inRange(saturation_matrix, target_saturation-t_sat, target_saturation+t_sat)
        masks.append(mask)
    if target_brightness:
        brightness_matrix = hsv_image[:,:,2]
        mask = cv2.inRange(brightness_matrix, target_brightness-t_brt, target_brightness+t_brt)
        masks.append(mask)
    for mask in masks:
        image = cv2.bitwise_and(image, image, mask=mask)
    return image

def edge_map(image):
    grayscale = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(grayscale, 10, 255)
    return edges

def find_centroids(corners):
    radius = 100
    maximum = corners.max()
    corners = cv2.threshold(corners, int(maximum*0.5), maximum, cv2.THRESH_BINARY)[1]
    corners = np.uint8(corners)
    temp = cv2.connectedComponentsWithStats(corners)
    return temp[3]

def corner_map(image):
    image = cv2.cornerHarris(image,5,11,0.04)
    return image

def find_shape(point_set, edges):
    tolerance = 0.1  # acceptable variation
    # create the initial set of vectors
    vectors = []
    point_set = [list(point) for point in point_set]
    point_set2 = [item for item in point_set]  # create copy
    for point1 in point_set:
        for point2 in point_set2:
            if ((point2[0] != point1[0]) and (point2[1] != point1[1])):  # verify that the points are not the same
                c_dist = point2[1] - point1[1]
                r_dist = point2[0] - point1[0]
                magnitude = int(np.sqrt(r_dist**2 + c_dist**2))
                unit_vector = ((r_dist/magnitude), (c_dist/magnitude))
                vectors.append(Vector(unit_vector, magnitude, point1, point2))
    print("vectors " + str(len(vectors)))
    # find vectors that correspond to edges

    # check for pairs that share a point
    vectors2 = [item for item in vectors]
    vector_pairs = []
    for vector in vectors:
        for vector2 in vectors2:
            if (vector.point_set[1] == vector2.point_set[0]):
                if not(vector.equals(vector2)):
                    vector_pairs.append((vector, vector2))
    print("pairs " + str(len(vector_pairs)))
    # check for perpendicular pairs
    temp = []
    for pair in vector_pairs:
        if Vector.is_ortho(pair[0], pair[1], tolerance):
            temp.append(pair)
    vector_pairs = temp
    print("pairs " + str(len(vector_pairs)))
    # check for overlapping pairs
    candidates = []
    for pair1 in vector_pairs:
        for pair2 in vector_pairs:
            if not((pair1[0].equals(pair2[0]) and pair1[1].equals(pair2[1])) or (pair1[0].equals(pair2[1]) and pair1[0].equals(pair2[1]))):  # they are not equal
                if pair1[0].equals(pair2[0]) and (abs(float(pair1[1].magnitude - pair2[1].magnitude) / pair1[1].magnitude) < 0.1):
                    candidates.append((pair1, pair2))
                    continue
    # check for correct shape
    final_results = []
    for item in candidates:
        points = []
        for pair in item:
            for vector in pair:
                for point in vector.point_set:
                    points.append(point)
        c_values = [point[1] for point in points]
        r_values = [point[0] for point in points]
        r_min, r_max = min(r_values), max(r_values)
        c_min, c_max = min(c_values), max(c_values)
        top_left = (r_min, c_min)
        dimensions = (r_max-r_min, c_max-c_min)
        final_results.append((top_left, dimensions))
    # select a candidate
    if len(final_results) > 0:
        final_results = final_results[0]
    return final_results

def test_img(img):
    hsv_img = color_filter(img, target_hue=12, t_hue=200)
    #edges = edge_map(hsv_img)
    edges = []
    gray_img = cv2.cvtColor(hsv_img, cv2.COLOR_BGR2GRAY)
    corners = corner_map(gray_img)
    #plt.gray()
    #plt.imshow(gray_img)
    #plt.show()
    centroids = find_centroids(corners)
    #plt.plot(centroids)
    results = find_shape(centroids, edges)
    #plt.imshow(edges)
    #plt.show()
    return json.dumps(results)

############### COMMUNICATION ################
data_buffer = None  # allows communication between publisher and subscriber

def process_img(img):
    global data_buffer
    bridge = cv_bridge.CvBridge()
    img = bridge.imgmsg_to_cv2(img)  # converts image to a format compatible with OpenCV
    img = np.uint8(img)  # keeps OpenCV happy
    data_buffer = test_img(img)

def start_gateway_detector():
    global data_buffer
    # start listener
    rospy.init_node('gateway_detector')
    rospy.Subscriber(camera, ROS_img, callback=process_img)
    # start publisher
    publisher = rospy.Publisher(destination, ROS_string, queue_size=3)
    rate = rospy.Rate(4)
    while not rospy.is_shutdown():
        if data_buffer is not None:
            publisher.publish(data_buffer)
            print data_buffer
        data_buffer = None  # reset
        rate.sleep()

############### MAIN PROCEDURE ###############

if __name__ == '__main__':
    start_gateway_detector()
