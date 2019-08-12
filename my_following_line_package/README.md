# To Launch

# Notes

## Applying Filters to the Image

The raw image is useless unless you filter it to only see the color you want to track, and you crop the parts of the image you are not interested in. This is to make your program faster.

We also need to extract some data from the images in a way that we can move the robot to follow the line.

### First step: Get Image Info and Crop the image

Before you start using the images for detecting things, you must take into account two things:

* One of the most basic pieces of data that you need to work with images are the **dimensions**. Is it 800x600? 1200x1024? 60x60?
This is crucial to positioning the elements detected in the image.
* And second is **cropping** the image. It's very important to work as soon as possible with the minimum size of the image required for the task. This makes the detecting system mush faster.

How do we determine what to crop? Well, it depends on the task. In this case, you are interested in lines that aren't too far away from the robot, nor too near. If you concentrate on lines too far away, it won't follow the lines, but it will just go across the map. On the other hand, concentrating on lines that are too close won't give the robot time to adapt to changes in the line.

It's also vital to optimize the region of the image as a result of cropping. If it's too big, too much data will be processed, making your program too slow. On the other hand, it has to have enough image to work with. At the end, you will have to adapt it to each situation.
