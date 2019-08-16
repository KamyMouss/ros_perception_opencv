# To Launch

# Notes
One of the most useful perception skills is being able to recognize objects. This allows you to create robots that can, for instance, grasp objects or understand the world around them a little bit better.
There are two main elements:

* **Recognize flat surfaces**: This skill allows it to detect places where objects normally are, like tables and shelves.It's the first step to searching for objects.
* **Recognize objects**: Once you know where to look, you have to be able to recognise different object in the scene and localise where they are from your robot's location.

The first step in recognising objects is knowing where these objects are. For this, you are going to use a port of the **object_recognition_core** package to be able to detect flat surfaces and represent that detections in RViz.

The package uses a binary called detection with a configuration file as the argument. This file, called *detection.tabletop_fetch.ros.ork*, in the directory *conf*, is where all the input sensors and values for the table detection are set. It's a YAML file with a different extension, .ork.

Of all the parameters you have here, the only ones that are relevant most of the time are the following ones:

rgb_frame_id: '/head_camera_rgb_optical_frame'
rgb_image_topic: '/head_camera/rgb/image_raw'
rgb_camera_info: '/head_camera/rgb/camera_info'
depth_image_topic: '/head_camera/depth_registered/image_raw'
depth_camera_info: '/head_camera/depth_registered/camera_info'

This sets the correct image topics as inputs so that the recognition can be made

And now, open the RVIZ and add all of the elements you want to see (like the Camera element or PointCloud2 elements). To visualize the Table detection, you will have to add **OrkTable** element. You select the topic where the table data is published, in this case, **/table_array**. You can then check certain options, like "bounding_box" to have a bounding box around the detection, or the "top" option to see what is being considered as the top of the surface.

So, once you have your session or your images stored, you need to be able to always start an object recognition session with all of that stored data. To do so, you have the following options:

For saved sessions in my_object_recognition_pkg, in the directory saved_pictures2d:

Now you can add as many objects as you want, or even turn the object around and take images from different points of view. This will make it detect the object all the time, but keep in mind that without the proper filtering, the system will consider them to be different objects.

So, the last step is to save all of the objects added. There are 2 main ways:

* Saving the Objects as images: File-->Save_Objects. This will save all of the images taken in a folder
* Saving the Whole session: File-->Save_Session. This will save a binary with all of the images and settings. This is the most compact way of doing it, although you won't have access to the images of the objects. It depends on your needs.

You can also save the object images directly by right clicking on them, but *BEWARE* that if you do so, these images can't be used for later detection because they aren't mirrored, therefore, the recognition won't work. You can see an example of it here. It's the same image, but one is the manually saved version that is not mirrored and isn't detected, while the other one is correctly saved.

So, once you have your session or your images stored, you need to be able to always start an object recognition session with all of that stored data. To do so, you have the following options:

### For saved sessions in my_object_recognition_pkg, in the directory saved_pictures2d:

<launch>
    <arg name="camera_rgb_topic" default="/head_camera/rgb/image_raw" />
	<!-- Nodes -->
	<node name="find_object_2d" pkg="find_object_2d" type="find_object_2d" output="screen">
		<remap from="image" to="$(arg camera_rgb_topic)"/>
		<param name="gui" value="true" type="bool"/>
		<param name="session_path" value="$(find my_object_recognition_pkg)/saved_pictures2d/coke_session.bin" type="str"/>
		<param name="settings_path" value="~/.ros/find_object_2d.ini" type="str"/>
	</node>

</launch>
<launch>
    <arg name="camera_rgb_topic" default="/head_camera/rgb/image_raw" />
    <!-- Nodes -->
    <node name="find_object_2d" pkg="find_object_2d" type="find_object_2d" output="screen">
        <remap from="image" to="$(arg camera_rgb_topic)"/>
        <param name="gui" value="true" type="bool"/>
        <param name="session_path" value="$(find my_object_recognition_pkg)/saved_pictures2d/coke_session.bin" type="str"/>
        <param name="settings_path" value="~/.ros/find_object_2d.ini" type="str"/>
    </node>
​
</launch>

### For saved images in my_object_recognition_pkg, in the directory saved_pictures2d:

<launch>
    <arg name="camera_rgb_topic" default="/head_camera/rgb/image_raw" />
    <!-- Nodes -->
    <node name="find_object_2d" pkg="find_object_2d" type="find_object_2d" output="screen">
        <remap from="image" to="$(arg camera_rgb_topic)"/>
        <param name="gui" value="true" type="bool"/>
        <param name="objects_path" value="$(find my_object_recognition_pkg)/saved_pictures2d" type="str"/>
        <param name="settings_path" value="~/.ros/find_object_2d.ini" type="str"/>
    </node>
​
</launch>

More Info:
http://wiki.ros.org/find_object_2d
http://introlab.github.io/find-object/