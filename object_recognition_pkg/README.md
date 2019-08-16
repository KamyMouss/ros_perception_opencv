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

So, once you have your session or your images stored, you need to be able to always start an object recognition session with all of that stored data. To do so, you have the following options:

### For saved sessions in my_object_recognition_pkg, in the directory saved_pictures2d:

See *saved_session_2d.launch*.

### For saved images in my_object_recognition_pkg, in the directory saved_pictures2d:

See *saved_objects_2d.launch*.

More Info:
http://wiki.ros.org/find_object_2d
http://introlab.github.io/find-object/

### Move and Spawn objects
You have to test your object_recognition system with the same object in different positions, or even in movement.
You also need to test it with various objects in the scene to be sure that it doesn't mistake a human head with a coke.

To make an object move in a scene, there are 2 steps:

* **Make the object movable in the Gazebo:** For that, you will use the move_model.launch from our spawn_robot_tools_pkg.This makes a topic called "name_object/cmd_vel" available on which you can then publish and move the model around.
* **Publish in the correct topic:** To move the object through the keyboard

See *make_cokecan_movable.launch* and *move_coke_keyboard.launch*.

To move other objects in the scene, just change the name of the model.
To know the name of that the model has in the Gazebo, you can ask the gazebo service:
`rosservice call /gazebo/get_world_properties`

### Spawn new objects in the scene
This method needs you to have the sdf files of the models on your package and the whole model installed in the .gazebo/models path. These models are in the *models* directory of this package.

Execute the *spawn_coke.launch* launch file to spawn the model where you wish, probably in the table location.
For reference, the coke_can is spawned in the center of the table at XYZ = (-2.0,0.0,0.8).
Bear in mind that if there is already an existing model with the same name, you won't be able to spawn it.

So, in conclusion, to spawn an object, for example the coke_can, and make it movable at the same time, you will have to execute *spawn_ready_coke.launch*.

## 3D Object Detection
The only real difference with the 2D detection will be the sensors involved and the fact that the ObjectPoseStamped will be transformed into TFs. 

You have to create another round of session photos in the 3D system because, otherwise, the detections won't work as well as they should. Especially for the TF transformations.

