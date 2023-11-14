# Data Format
Trajectory 1 contains a json file that has the following keys:

* __fl_x__ 
The focal length in the x direction given in pixels
* __fl_y__ 
The focal length in the y direction given in pixels
* __cx__ 
The x coordinate of the center of the image in pixels
* __cy__ 
The y coordinate of the center of the image in pixels
* __w__ 
The width of the image in pixels 
* __h__
The height of the image in pixels
* __integer_depth_scale__
The scale factor to convert the depth image to meters
* __frames__
A list of each of the images along the trajectory. It contains
    * __file_path__
        The path to the rgb image.
    * __depth_path__
        The path to the depth image. Depth is stored as a 16 bit png image. This means that each pixel is in the depth image is a number between 0 and 65535. To convert this to meters, multiply by the integer_depth_scale. If the number is 0, the depth is unknown.
    * __transform_matrix__
        The 4x4 transformation matrix that transforms points from the camera frame into the world frame.

Trajectory 2 contains the same information as trajectory 1, but the transformation matrix is not given 


