import cv2
import sys
print(sys.argv)



def write_to_file(img, filepath):
	# ROS Image message -> OpenCV2 image converter
	from cv_bridge import CvBridge, CvBridgeError
	bridge = CvBridge()

	cv2_img = bridge.imgmsg_to_cv2(img, "rgb8")
	cv2.imwrite(filepath, cv2_img)

	return 11111
