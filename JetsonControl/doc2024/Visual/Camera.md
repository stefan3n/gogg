### [< back](../GuideForDocumentation.md)
## We have used 2 USB cameras. It is the most simple way to use them. You can also use the serial cameras but they are more difficult to use to say the least.
## Use this command:
```sudo apt-get install v4l-utils```

## If no CSI cameras are involved just install opencv with:
```pip3 install opencv-python```
## If you want CSI support, you have to build opencv from source. Refer to [this/optional](AI.md)

## Note:
### We have not used CSI cameras because there was a big delay and could not get it to work flawlessly.

## To use cameras in python please refer to the opencv doc or our code. You just need a cv2.VideoCapture object. In the name field insert the /dev/videoX and in the cap field insert ```cv2.CAP_V4L2```.

## For more details refer to the code/code comments.