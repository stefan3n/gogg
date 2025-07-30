### [< back](../GuideForDocumentation.md)
# Grab a bottle
### We mounted the camera on the top of the arm. The camera gives the pixel coordinates of the bounding box (top left and bottom down). I determine the center of the bounding box. To move the arm we should transform the pixel coordinates to real world coordinates.
### We need to do some measurement:
- The length of the links including camera center (this is an approximate measurement).
- The distance from the camera to the ground.
- How much is a pixel in the real world:
  * Use a roulette and measure the width of the camera in cm (put it on the ground and see how much camera can see).
  * After determine the resolution of the camera.
  * Do a math calculus to find out how much is a pixel.
### After I obtained this information I started to calculate the coordinates in the real world. First I need to do some translations of the axis because we obtain the relative coordinates to the camera and we need them in relative to the first join. For this to determine the distance from this point I use the measurements I did and I calculate the distance from the camera and the first join, after I add the distance from the camera to the object. Because the arm is some height from the ground I send for height always value 0, because that is the height I need. Using atan function from math I can calculate how much I need to rotate the arm in a direction. But this value is relative to the one margin of the field of view. To solve this I subtract the middle value, apply at this value abs function. To determine the direction for the arm rotation I check where it is positioned relative to the middle position for the x axis. To determine the approximate how much I need to rotate the claw I calculate the height and width ratio and see if it is bigger, smaller or equal to 1. After this calculus I send the command to move the arm with these parameters.
##### To determine the pixel dimensions this [link](https://automaticaddison.com/how-to-convert-camera-pixels-to-real-world-coordinates/) can help.
