## To Launch
See notes for the various applications.

## Notes

It is important to be able to make a robot follow people around and detect when they approach so that you can avoid them or stop. This skill is absolutely essential for robots in public spaces, like museums and malls. It's also important in HRI because a robot has to be able to follow a person, like its instructor, to go wherever it's needed.

## ROS package for tracking people

There are various ways in which a person can be tracked:

**Detecting legs**: This is the most basic way of detecting people. You just look for laser patterns that have a U shape. As you might guess, this gives a lot of false positives, especially when there are chairs around because they get easily confused by the legs.
**Detecting upper body**: This method detects people looking for patterns that match upper body shapes. More robust than leg detection.
**Detecting pedestrians**: This is a merging of some data from ground_hog, visual_odometry, und upper_body_detector.

But, in the end, none of them are as strong as all of them combined. So the best algorithms are the ones that combine all of the data from these detecting systems and many others, and process them to have coherent detection.

## Leg Detector

To start the leg detector, you will use the **leg_detector** package. As input, it will take the **/base_scan** topic, which contains the laser readings' data.

See _leg_detector_start.launch_.

```xml
<arg name="scan" default="/base_scan" />
<arg name="machine" default="localhost" />
<arg name="user" default="" />
```

Here you are really just setting the machine and user to the default values. This is for ssh connections, but nothing that is relevant now.
What is relevant is the **scan** topic that, in this case, is **/base_scan**. This topic is where the laser readings are published. These readings are the ones used by the **leg_detector**.

```xml
<node pkg="leg_detector" type="leg_detector" name="leg_detector" args="scan:=$(arg scan) $(find leg_detector)/config/trained_leg_detector.yaml" respawn="true" output="screen">
    <param name="fixed_frame" type="string" value="odom" />
</node>
```

Here you are launching the **leg_detector**, taking the odom frame as the fixed_frame. But, the most important element here is the **trained_leg_detector.yaml** file. This is where all of the configuration parameters for the recognition are set.

```xml
<!-- To PoseArray -->
<include file="$(find detector_msg_to_pose_array)/launch/to_pose_array.launch">
    <arg name="machine" value="$(arg machine)"/>
    <arg name="user" value="$(arg user)"/>
</include>
```

This last launch is quite strange. Why do you need to launch this **to_pose_array.launch** file? To understand why, you should first launch it, and then take a look at the topics published.

`roslaunch people_tracking leg_detector_start.launch`

```
rostopic info /leg_tracker_measurements
rostopic info /to_pose_array/leg_detector
```

As you can see, these two topics publish position data. And the topic **/to_pose_array/leg_detector** is the one published by the to_pose_array.launch file. So, why do you need it? Basically, what this is doing is to transform the **/leg_tracker_measurements** data into **PoseArray**. This will be needed in the end, when you combine all detecting systems together. The only requirement for combining detectors is that all of them have to be in **PoseArray** format. So... because the leg detector doesn't generate that on its own, you have to tranform it.

Also, at the end of the pipeline, there is a topic called **/people_tracker_measurements**. This is where the final data will be published through all the people tracker systems.

## Detect Upper Body

See *upper_body_start.launch*.

This system detects the upper part of a human's body and publishes its estimated location and orientation. It basically publishes a PoseStamped with position and orientation. It additionally publishes the RGB image with a square overlapping where the person is detected.

As you can see, this gets a bit more messy. But, essentially, you have to launch two things:

* Launch the **upper_body_detector.launch**: This is the one that makes the recognition.
* Launch the **ground_plane_estimated.launch**: This is totally necessary for the upper_body_detector.launch to work, because it's subscribed to the /ground_plane topic. Knowing where the ground is allows it to calculate and detect the distance to where the upper body should be and look around there.

The topics for the image are pointed out:

```xml
<arg name="camera_namespace" value="/head_camera" />
<arg name="rgb_image" value="/rgb/image_raw" />
<arg name="depth_image" value="/depth_registered/image_raw" />
<arg name="mono_image" value="/depth_registered/image_raw" />
<arg name="camera_info_rgb" value="/rgb/camera_info" />
<arg name="camera_info_depth" value="/depth_registered/camera_info" />
```

As you can see, they are divided into their main elements. If you want to use this with other robots, you will obviously have to change this part.
So, when you launch **upper_body_start.launch**, you will get a bunch of topics related to the **upper_body** node, but these are the ones that are relevant for people tracking:

```
/upper_body_detector/bounding_box_centres                                                                           /upper_body_detector/closest_bounding_box_centre                                                                   
/upper_body_detector/detections                                                                                     
/upper_body_detector/image                                                                                         
/upper_body_detector/marker_array
```

You have a **bounding_box_center** and a **closest_bounding_box_center** topics. These two are used to position the people detected in the space. The closest is used in case you want to follow the person that is closer to you from the start, for instance, and then follow them with other methods, in case of someone crossing between the robot and the person.

The **detections** topic provides the person detections that the system does, and the **marker_array** topic has this same data but in a marker format, which is very handy if you want a representation in RViz.

You can visualise the topic **/upper_body_detector/image**, selecting it in **rqt_image_view**.

`rosrun rqt_image_view rqt_image_view`

**NOTE:** As you can see, there is no effect on which way it's facing. Although it has problems when detecting a person sideways. So keep that in mind.

## Pedestrian detector:

See *mdl_pedestrian_start.launch*.

This pedestrian detector uses the **detection of the ground** to filter false positives, and other algorithms to filter even more and make better predictions, including the **upper body** detections.

Although it might seem enormous, you can see that everything, except a few things related to the **mdl_people_tracker.launch**, is exactly the same as what you already know. That's because this mdl works on the previous data of UpperBody and ground, plus using visual_odometry.

When you execute this, you will get a few topics. These are the most relevant:

```
/mdl_people_tracker/image                                                                                           /mdl_people_tracker/marker_array                                                                                   
/mdl_people_tracker/people_array                                                                                   
/mdl_people_tracker/pose_array
```

As before, you have some topics publishing Poses, and other topics publishing markers which contain this Pose information as markers in order to be able to visualize it in RVIZ.

You can also visualize the topic **/mdl_people_tracker/image**, in this case, with a cylinder around the detection.

**NOTE:** Observe that if it loses the person, the cylinder will probably change its color, considering it another person. So keep that in mind.

## Combining it all together
And, finally, once you have all of these systems working, you are now going to use all of the information and process it with the package **spencer_people_tracking**.
To launch it, one of the most important things is the configuration file, in which you add all of the so-called **detectors**, plus the detection by speed model.
You unite all of this information through a [Kalman filter](https://en.wikipedia.org/wiki/Kalman_filter), more precisely a [Unscentered Kalman Filter](https://www.seas.harvard.edu/courses/cs281/papers/unscented.pdf). Kalman filters are used to extract conclusions from sets of data that, by themselves, are inaccurate, but combined with other data sources, are more precise.

See *detectors.yaml*.

The most important value here is the **cartesian_noise_params**. This will regulate the impact in the final decision that the detector makes. And which values do you have to state here?
Well, that's a good question. There are two methods that you can follow:

- **Extract the statistical values of each detector system:** This method consists of executing only that detector system in a controlled environment. You, then, save all the X and Y values of the pose array. The more you have, the better the statistical values that you will get. Once done, generate the mean and standard deviation of the X and Y separately. You will end up with a Standard Deviation for X and a Standard Deviation for Y. **These values are the ones you will have to use as cartesian_noise_params**.
- Start with the example values provided and change them until you get the best results. This way of working, although not at all scientific, its very common in engineering testing. Sometimes, you are much faster with this method, just iterating over values and selecting the best results, rather than extracting all the theoretical data or models. This is due to the fact that you may not have all the real elements in mind when you create your theoretical tests.

The launch will be divided into two launches for more flexibility in changing arguments. The first one, *people_tracker_custom.launch*, is the core. Here, all the systems are launched. This includes: 
- visual_odometry
- ground_plane_estimation
- upper_body_detector
- pedestrian tracker (mdl_people_tracker)
- leg_detector
- people_tracker
- logging

The second launch, *start_peopletracker_custom.launch*, only remaps the values to your particular case.

Once you launch the *start_peopletracker_custom.launch* file, you will have access to the following topics. Be aware that sometimes it may take a few seconds for the topics to start publishing meaningful data, so be patient:

* /people_tracker/marker_array: Position and orientation in Marker Format
* Topics related to the people detected and positions:

```
/people_tracker/people                                                                                         
/people_tracker/pose
/people_tracker/pose_array
/people_tracker/positions
```

And these markers have the final positions of the detections of the people. But, there is more. You can visualize these markers in RViz, so let's take a look. To add them, you have to add a **MarkerArray** element and select the marker topic.

You can also add some extra elements, like the laser readings and the camera, to see the detection markers imposed and how precise they are. You can also add the image of the mlc, for example, to keep track.

More information here: https://github.com/spencer-project/spencer_people_tracking
