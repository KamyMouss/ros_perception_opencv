# To Launch

# Notes
One of the most useful perception skills is being able to recognize objects. This allows you to create robots that can, for instance, grasp objects or understand the world around them a little bit better.
There are two main elements:

* **Recognize flat surfaces**: This skill allows it to detect places where objects normally are, like tables and shelves.It's the first step to searching for objects.
* **Recognize objects**: Once you know where to look, you have to be able to recognise different object in the scene and localise where they are from your robot's location.

The first step in recognising objects is knowing where these objects are. For this, you are going to use a port of the **object_recognition_core** package to be able to detect flat surfaces and represent that detections in RViz.

The package uses a binary called detection with a configuration file as the argument. This file, called *detection.tabletop_fetch.ros.ork*, in the directory *conf*, is where all the input sensors and values for the table detection are set. It's a YAML file with a different extension, .ork.

Of all the parameters you have here, the only ones that are relevant most of the time are the following ones:

* rgb_frame_id: '/head_camera_rgb_optical_frame'
* rgb_image_topic: '/head_camera/rgb/image_raw'
* rgb_camera_info: '/head_camera/rgb/camera_info'
* depth_image_topic: '/head_camera/depth_registered/image_raw'
* depth_camera_info: '/head_camera/depth_registered/camera_info'

This sets the correct image topics as inputs so that the recognition can be made

And now, open the RVIZ and add all of the elements you want to see (like the Camera element or PointCloud2 elements). To visualize the Table detection, you will have to add **OrkTable** element. You select the topic where the table data is published, in this case, **/table_array**. You can then check certain options, like "bounding_box" to have a bounding box around the detection, or the "top" option to see what is being considered as the top of the surface.
