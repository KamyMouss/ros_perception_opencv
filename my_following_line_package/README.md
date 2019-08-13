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

### Second step: Convert from BGR to HSV
Remember that OpenCV works with BGR instead of RGB, for historical reasons (some cameras, in the days when OpenCV was created, worked with BGR).
Well, it seems that it is not very easy to work with either RGB or BGR, to differentiate colors. That's why HSV is used. The idea behind HSV is to remove the component of color saturation. This way, it's easier to recognise the same color in different light conditions, which is a serious issue in image recognition. More information:

### Third step: Apply the mask
Now, you need to generate a version of the cropped image in which you only see two colors: black and white. The white will be all the colors you consider yellow and the rest will be black. It's a binary image.

Why do you need to do this? It basically has two functions:

* In doing this, you don't have continuous detection. It is the color or it's NOT, there is no in-between. This is vital for the centroid calculation that will be done after, because it only works on the principal of YES or NO.
* Second, it will allow the generation of the result image afterwards, in which you extract everything on the image except the color line, seeing only what you are interested in seeing.

### Fourth step: Get The Centroids, draw a circle where the centroid is and show all the images
Centroids, in essence, represent points in space where mass concentrates; the center of mass. Centroid and centers of mass are the same thing as far as this course is concerned. And they are calculated using Integrals.

This is extrapolated into images. But, instead of having mass, we have color. The place where there is more of the color that you are looking for is where the centroid will be. It's the center of mass of the blobs seen in an image.

That's why you applied the mask to make the image binary. This way, you can easily calculate where the center of mass is located. This is because it's a discrete function, not a continuous one. This means that it allows us to integrate in a discrete fashion, and not need a function describing the fluctuations in quantity of color throughout the region.

These centroids are vital in blob tracking because they give you a precise point in space where the blob is. You will use this to follow the blob and, therefore, follow the line.

### Note on image moments (used for centroid calculation)
In image processing, computer vision and related fields, an image moment is a certain particular weighted average (moment) of the image pixels' intensities, or a function of such moments, usually chosen to have some attractive property or interpretation.

Image moments are useful to describe objects after segmentation. Simple properties of the image which are found via image moments include area (or total intensity), its centroid, and information about its orientation.
