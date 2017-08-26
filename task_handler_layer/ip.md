# IMAGE PIPELINE

## PRE-PROCESSING

We use opencv-2.4.9 library for doing all of our image processing tasks.

Pre-processing  process  filters  out  any  noise  that  might  occur  due  to  changes  in  water  and  lighting conditions. It  essentially  includes  all the image enhancement  techniques  such as contrast correction, white balancing, noise reduction  etc.

This contains all the filters which are applied to improve the quality of underwater images and noise reduction techniques for reducing the colored noise, then thesholding it. We need to extracrt every part  of  the  image  that  is  important  w.r.t  to  the  task, i.e. the hightlighted picture of the targets, is  brought  to  the foreground and the rest goes into the background.

Since the underwater images are degraded so the image the image enhancement techniques are applied to make the target clearly visible, colors distinguinsable, and a little color unformity for easy thresholding.

### Original Image
<p align="center">
  <img src="https://github.com/ksakash/my_cv/blob/master/test_images/frame0000.jpg?raw=true" alt="Sublime's custom image"/>
</p>

## CLAHE algorithm for color correction

Ordinary histogram equalization uses the same transformation derived from the image histogram to transform all pixels. This works well when the distribution of pixel values is similar throughout the image. However, when the image contains regions that are significantly lighter or darker than most of the image, the contrast in those regions will not be sufficiently enhanced.

[Adaptive histogram equalization (AHE)](https://en.wikipedia.org/wiki/Adaptive_histogram_equalization) is a computer image processing technique used to improve contrast in images. It differs from ordinary histogram equalization in the respect that the adaptive method computes several histograms, each corresponding to a distinct section of the image, and uses them to redistribute the lightness values of the image. It is therefore suitable for improving the local contrast and enhancing the definitions of edges in each region of an image.

However, AHE has a tendency to over-amplify noise in relatively homogeneous regions of an image. A variant of adaptive histogram equalization called contrast limited adaptive histogram equalization (CLAHE) prevents this by limiting the amplification.

Then the bilateral filter is applied to remove the colored noise preserving the edges.

### After color correction
<p align="center">
  <img src="https://github.com/ksakash/my_cv/blob/master/test_images/1.jpg?raw=true" alt="Sublime's custom image"/>
</p>

## White balance for removing color casts

[White balancing](https://gist.github.com/tomykaira/94472e9f4921ec2cf582) is an important processing step that aims to enhance the image appearance by discarding unwanted color casts, due to various illuminant.

After this bilateral filter is again applied to remove the noise.

### After removing color cast
<p align="center">
  <img src="https://github.com/ksakash/my_cv/blob/master/test_images/2.jpg?raw=true" alt="Sublime's custom image"/>
</p>

To reduce the noise in thresholded image Image Morphology is used. Image morphology is often done on binary images that result from thresholding.
The [`dilation()`](http://docs.opencv.org/2.4/doc/tutorials/imgproc/erosion_dilatation/erosion_dilatation.html/)
function is applied which computes a local maximum over the area of the kernel.
In general, whereas dilation expands region A, [`erosion()`](http://docs.opencv.org/2.4/doc/tutorials/imgproc/erosion_dilatation/erosion_dilatation.html) reduces region A. Moreover, dilation will tend to smooth concavities and erosion will tend to smooth away protrusions. So, combination of erosion and dilation operations are used to reduce the noise according to the specific tasks.

### After thresholding
<p align="center">
  <img src="https://github.com/ksakash/my_cv/blob/master/test_images/frame0001.jpg?raw=true" alt="Sublime's custom image"/>
</p>

The [`findContours()`](http://docs.opencv.org/2.4/modules/imgproc/doc/structural_analysis_and_shape_descriptors.html?highlight=findcontours#findcontours) function is used to find the contours in the image through which we will get all the relevant information about our targets like its center, contour area, coutour center, etc.

After this task specific algorithms are applied to get the information about the task like coordinates, contour area and center of contour.
