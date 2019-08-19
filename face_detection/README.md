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

Note that because you are using a simulation, you don't need to start the openni server, it's already done for you. In the case of the real robot, you will probably have to start it.