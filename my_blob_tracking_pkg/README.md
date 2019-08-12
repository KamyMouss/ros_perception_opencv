# To launch
`roslaunch my_blob_tracking_pkg my_mira_cmvision_tc.launch`
# Notes
We can use the package **cmvision** for blob tracking:

## Tracking color gui:

`rosrun cmvision colorgui image:=<image topic>`


You have the RGB value of the color on average and then you have the YUV, which, in this case, is (30:82, 86:111, 178:252). Yours could be slightly different. But, the point is that this YUV defines the color blob to be tracked. This means that this is what MiraRobot will consider to be RedBall.

in colors.txt, you see that there are two parts: Colors and Thresholds.

## In Colors, you state:

The RGB value of the line around the detection, in this case, is the same color as the average RGB color detected previously, but you can put any color you want. It's just good practice to put the Average color because, that way, you can see what blob is being detected.
The other two numbers are not used in this version of cmvision.
The name of the blob representation, in this case RedBall

## In Thresholds, you state:

Essentially, the Red, Green, and Blue value range. In the case for the RedBall, Red = [from 30 to 81], Green = [from 86 to 111], and Blue = [from 178 to 253]
All of them are placed in the same order as the Color List to make the correspondence.
As you can see, you can put as many colors as you wish and, afterwards, be able to track different blobs based on their names, like Green, RedBall, Teal, etc.

## /blobs topic
cmvision publishes blob topic.

Each cmVision/blob message provides the information for a single detected blob.

As you can see, you will get a lot of information out of each detected blob. Anyways, the most important information is the blob name, the position in the image, and the size. With this, you can position the blob in a 2D, and maybe in a 3D space. You should also be able to track multiple objects in a 3D space, and know which blob it is each one, if they are sufficiently different in colour.

You should see the Mira camera with the tracking system already working in the Graphical Interface. If you don't have a red box around the ball, you should check that the YUV values you gave are enough, and run the color setup again, if necessary.
