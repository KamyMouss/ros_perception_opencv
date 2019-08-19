# To launch

`roslaunch face_detector face_detection_tc.launch`

# Note

There are three main elements to face detection:

- **Detect Faces**: This means that it can detect where a human face is and address it for talking or giving feedback. But it can't know who it is, it just knows that it's a human face. This is important for emotion reading, listening to orders, or just giving certain feedback that is so necessary for Human Robot Interaction, or HRI.
- **Recognise Faces**: Now this skill allows the robot to know who he is talking to. This is vital for security purposes or just to know how to address each individual.
- **Track People**: The robot has to be able to track the movements of humans, be able to follow the one it needs to follow, and filter the others, without colliding with them.

## Face Detector in ROS
You will use the package **face_detector**.

See: *face_detection_tc.launch*

The setting of the arguments is vital for this to work. It's divided into basic elements of the topics so that it can use all of the topics that you have from the PointCloud Camera.

The face detection uses two types of data:

- **RGB image**: This is published, in this case, in the topic **/head_camera/rgb/image_raw**
- **Depth Image Data**: This is published in the topic **/head_camera/depth_registered/image_raw**
These topics, then, have to be divided into the corresponding arguments, resulting in setting them the way mentioned.
The **fixed_frame** selected is the one where the PCL camera is. In this case, it's the **head_camera_rgb_optical_frame**. Normally, it's always the optical frame of the robot that is selected.

_Note that because you are using a simulation, you don't need to start the openni server, it's already done for you. In the case of the real robot, you will probably have to start it._

For more details, refer to: http://wiki.ros.org/face_detector.


## Face Detector Client

The server will only publish if there is a client connected to it. This is quite common in well-designed servers to avoid overflooding the ROS system with data that no one is listening to.

The *face_detector_client.py* will subscribe to the **/face_detector/people_tracker_measurements_array** topic, and all the face detecting topics will have to start publishing data. This topic will get the position of every face detected, with an ID for each one. Perfect for tracking many people at the same time.

## Visualise the Face detections
Visualising the detection data is crucial in order to understand if the robot is really comprehending what's going on around it. That's why you have to use RViz and special markers to indicate where the face detections are made and when they stop.

Launch RViz and load the RViz config file in the *rviz_config* directory.

This topic representation ( **/face_detector/people_tracker_measurements_array** ) is giving you the position of the face detected. As you can see, it is placed more or less where the person frame TF is.
Let's have a look at the topics read for getting this information, so that you can reproduce this anywhere:

* The first thing to note is that this is not your run-of-the-mill RViz marker. This is because it's using the jsk_rviz_plugins package. You will have to install this package if you want to use it. These markers give a lot of functionality to RViz and have many applications.
* The LaserScan and the PointCloud2 data are disconnected to avoid overflooding the PC, because PCL consumes quite a lot of resources. But you can activate them by simply checking the box.
* PeoplePositionMesurementsArray: This is the blue circle drawn around the position of the face detected. It's the main data we are looking for.
* Camera: Just the RGB camera, as a reference, which RViz superimposes the PeoplePositionMesurementsArray data.


