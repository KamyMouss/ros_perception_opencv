## To launch

Run the following script to move the robot around:
`rosrun face_recognition move_fetch_robot_client.py`

Run the face recognition script:
`rosrun face_recognition recognize_face.py`

## Notes

For this, you will use the **face_recognition** python module.

It has many features apart from the FaceRecognition feature, like face finding or applying digital makeup. But it gives a very simple way of doing a rudimentary face recognition. Here you have an example of how to recognise _one face_.

# Comments on recogize_face.py

```
import cv2
import face_recognition
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import rospkg
```

You will need these special packages, like **cv2** (OpenCV), **cv_bridge** (CV_Bridge), and **face_recognition** to be able to recognize the faces. You will also need rospkg in order to easily find files that are inside other ROS packages.

```
# get an instance of RosPack with the default search paths
rospack = rospkg.RosPack()
# get the file path for face_recognition_pkg
self.path_to_package = rospack.get_path('face_recognition_pkg')
print self.path_to_package
```

Then, you need to **retrieve an image** of the person that you want to recognise. **It is important to note** that by giving only one image, the face recognition system won't make positive detections unless what it visualizes is very similar to the image provided. So, the more images you provide using different configurations, the better the detections will be. Anyways, one image is more than enough.

```
# Load a sample picture and learn how to recognize it.
image_path = os.path.join(self.path_to_package,"scripts/standing_person.png")
```

Once you have that, it starts the real image processing:

```
standing_person_image = face_recognition.load_image_file(image_path)
standing_person_face_encoding = face_recognition.face_encodings(standing_person_image)[0]
```

Here you see that you load the image into a face_recognition variable, and then you extract the encodings. Note that you are getting the first encoding [0]. This is because you only have one image of that person. More images will generate more encodings.

We initialise some variables and then we underscale the image retrieved from the cameras. In this case, half of the original size. That way, you work with less pixels, making the recognition process faster. But, when you show the images captured, you will show the original size.
But this has a side effect: if you reduce the **size too** much, the face encoder has less information to work with and, therefore, it makes it harder to extract human features and match them with someone.
It has to be a compromise between performance and functionality. If you are too far away from your subjects, then you will have to work with bigger images.

With smaller image, it will be difficult for the algorithm to work, so you will have to get closer.

```
# Initialize some variables
face_locations = []
face_encodings = []
face_names = []
process_this_frame = True

# Resize frame of video to 1/2 size for faster face recognition processing
small_frame = cv2.resize(video_capture, (0, 0), fx=0.5, fy=0.5)
```

Then we process the image frame. Bear in mind that, here, you are processing every other frame. This, again, is to reduce the number of images to be processed, making the recognition faster.

```
# Only process every other frame of video to save time
if process_this_frame:
    # Find all the faces and face encodings in the current frame of video
    face_locations = face_recognition.face_locations(small_frame)
    face_encodings = face_recognition.face_encodings(small_frame, face_locations)

    face_names = []
    for face_encoding in face_encodings:
        # See if the face is a match for the known face(s)
        match = face_recognition.compare_faces([standing_person_face_encoding], face_encoding)
        name = "Unknown"

        if match[0]:
            print "MATCH"
            name = "StandingPerson"

        face_names.append(name)

process_this_frame = not process_this_frame
```

First, extract the location of the faces. Then, from those locations, you extract the encodings. These encodings are like the "fingerprint" of each face, defining it. You do this for each image frame that you capture.
Then, for all the faces detected and, therefore, encodings, you compare those encodings with the standing_person_face_encoding. If there is a match, then you found the standing person, and you add that name to the face_names list.

Finally, you display the results of the face recognition. This means showing the image captured and the locations where you detected a face, and draw a square with the name of the recognised face there.

```
# Display the results
for (top, right, bottom, left), name in zip(face_locations, face_names):
    # Scale back up face locations since the frame we detected in was scaled to 1/2 size
    top *= 2
    right *= 2
    bottom *= 2
    left *= 2

    # Draw a box around the face
    cv2.rectangle(video_capture, (left, top), (right, bottom), (0, 0, 255), 2)

    # Draw a label with a name below the face
    cv2.rectangle(video_capture, (left, bottom - 35), (right, bottom), (0, 0, 255))
    font = cv2.FONT_HERSHEY_DUPLEX
    cv2.putText(video_capture, name, (left + 6, bottom - 6), font, 1.0, (255, 255, 255), 1)

# Display the resulting image
cv2.imshow("Image window", video_capture)
cv2.waitKey(1)
```

The result of this should be a frame showing frames with labels around recognized faces.

## Multiple Face Recognition at the Same Time

Similar to single face recognition, but import multiple images.

See _recognize_multiple_faces.py_.
