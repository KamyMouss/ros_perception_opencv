## Start The Face Recognition Package

For this, you will use the **face_recognition** python module.

It has many features apart from the FaceRecognition feature, like face finding or applying digital makeup. But it gives a very simple way of doing a rudimentary face recognition. Here you have an example of how to recognise _one face_.

## Notes

# Comments on recogize_face.py

`import cv2 import face_recognition from cv_bridge import CvBridge, CvBridgeError from sensor_msgs.msg import Image import rospkg`

You will need these special packages, like **cv2** (OpenCV), **cv_bridge** (CV_Bridge), and **face_recognition** to be able to recognize the faces. You will also need rospkg in order to easily find files that are inside other ROS packages.

Then, you need to **retrieve an image** of the person that you want to recognise. **It is important to note** that by giving only one image, the face recognition system won't make positive detections unless what it visualizes is very similar to the image provided. So, the more images you provide using different configurations, the better the detections will be. Anyways, one image is more than enough.

`# get an instance of RosPack with the default search paths
rospack = rospkg.RosPack()
# get the file path for face_recognition_pkg
self.path_to_package = rospack.get_path('face_recognition_pkg')
print self.path_to_package`
