# README

# g2o may get comflicts
Because there existing some comflicts between the old version and new version.
So if error shows up about g2o during the make process please install the g2o version in 3rdparty folder.

# Usage
## in the dia folder:  
mkdir build && cd build  
cmake ..  
make  

## in the dia/condig/default.yaml
change the **dataset_dir** to your own path.  
The experiments was done only on **freiburg1_xyz** dataset

# Explanation of the codes structure
## config 
read the parameters of dataset path and camera matrixes  
## camera 
store the intrisix matrixes of the camera and some methods to transform the points between different coordinates.  
## frame
store the color and depth for a image in a certain time_frame, 
includes the methods for calculating the jacobian and bilinear interpolation to get image intensiy given a pixel coordinates.  
## g2o_types
define class inherenting the Edge and Vertex form g2o.
## visual_odemtry
implementing direct image allienment by defferent optimizting methods form g2o and naive implemetnation of Gauss-Newton and GD
## pyframes
Build the image pyramids through naive downsample methods.

# Possible improvement
because of the limiting time, the result is not good enough, especially the naive implementation of my own optimization methods.
Possible improvement could be on such perspectives:  
1. **improve the image pyramids**  
Now the pyramids is build on the naive implementation of downsampling, better method could be develeped  
2. **using iamge patchs**  
instead of using single pixels coordinates, patch photometric errors could be a better method considering also the context around a single pxiel.
3. **eliminating the outliers during the iteration**  
there are lots of outliers during the iteration, which results in negative influence on the estimation. 
# Reference
**DVO_SLAM**   https://github.com/tum-vision/dvo_slam
**Dense Visual SLAM for RGB-D Cameras**
 https://vision.in.tum.de/_media/spezial/bib/kerl13iros.pdf




