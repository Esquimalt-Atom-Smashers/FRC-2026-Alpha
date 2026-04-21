# Introduction
Code resides on the Raspberry Pi with a specific installation (Raspberry Pi OS with virtual environment allowing modified Python binary).

On the Pi, this code is named "post-detect.py" (previously named "post-HSV-Depth-nt.py")

This will connect with the Roborio and publish a table 'PostDetection' that includes:
     if a post is detected (boolean) 
     the horizontal left/right (lateral) position in metres
     the depth (horizontal distance normal front of camera front) in metres

Connects with RealSense camera and:
   0. Images from RGB (colour) camera and LIDAR (depth) camera are aligned. The clipping distance is set (all pixels from 3D camera that are farther than clipping distance will be ignored)
   1. all pixels > clipping distance are set to black. This image is then cleaned using OpenCV morphologyEx: MORPH_CLOSE using Rectangular element.
The raw image from the RGB camera:<img width="319" height="273" alt="image" src="https://github.com/user-attachments/assets/6bec244b-dedd-4c40-8c1e-c12aa92dcb25" />

The initial mask from the depth camera:<img width="320" height="263" alt="image" src="https://github.com/user-attachments/assets/d7e90a09-01db-482d-b4b7-3b74b936928c" />

Lightly cleaned depth mask:<img width="320" height="267" alt="image" src="https://github.com/user-attachments/assets/ebda7f8e-7577-4b4d-9c6f-641a4d70bdcd" />


   2. depth mask is applied to colour image: <img width="320" height="274" alt="image" src="https://github.com/user-attachments/assets/be8b1bc3-3fda-4f09-a560-5e7ea903646d" />

   3. HSV colour mask is then applied: all pixels out of colour range (red or blue) are set to black
      All pixels in HSV range are set to light grey <img width="319" height="267" alt="image" src="https://github.com/user-attachments/assets/83b86f09-e6ae-42b1-9551-bf8b698cf985" />

The colour-mask applied to the depth-masked colour image from step 2 (Note: just for illustration, this isn't used by the program):
<img width="318" height="269" alt="image" src="https://github.com/user-attachments/assets/bef6731a-03c9-417a-b180-f71cf634860c" />

   4. OpenCV "contours" used to choose most post-like piece of image:
         - small contours are ignored as noise (<5 wide or <20 high)
         - only contours with minimum aspect ratio (height/width) greater than set value are considered
         - Note: minimum aspect-ratio was originally 2.0, but reduced to 1.6 after testing
         - contours are scored based on area, the largest area is selected
         - future improvement: include average depth of each part of 
         - centre of this contour is considered returned as the x-value in pixels
         - intrinsics are used to convert the x-value to metres
         - as camera gets closer to post, precision should increase as edge noise has less weight

The horizontal/lateral position of the post centre:<img width="320" height="272" alt="image" src="https://github.com/user-attachments/assets/52c27404-3ac6-43b6-8e71-3848a216e2d5" />

   5. Results are published to NetworkTable 'PostDetection'
         - 'post_detected': Boolean, True if post detected
         - 'post_lateral': Number, horizontal displacement perpendicular to depth-direction, in metres
         - 'post_depth': Number, distance from camera (normal to camera-face) in metres

At the time of writing, the plan is to use this camera and code for final alignment during climbing stage. Odometry and possibly turret-targeting cameras will be used to get "close" to the post (30 to 50cm away). Once the "Post Detection" NetworkTable boolen "post_detected" is TRUE, the RoboRIO climbing code will switch to using the "Post Detection" NetworkTable data, "post_lateral" and "post_depth" to accurately position so it can attach and climb.

