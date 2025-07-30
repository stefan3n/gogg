### [< back](../GuideForDocumentation.md)
# AI
## We have used the Jetson Nano Orin and python 3.8. Check the python version with ```python3 --version```.
## The toycar uses YOLOv10n which is a pretrained model. The model itself searches in each frame for bottles (we have not trained it on a custom object although it is possible). To use YOLOv10, you have to:
### Install CUDA
#### Look up [here](https://forums.developer.nvidia.com/t/pytorch-for-jetson/72048) the version of pytorch that is compatible with your system (we have used v2.1.0) and download the pip wheel. 
#### Enter [here](https://pytorch.org/get-started/previous-versions/) and find your pytorch version and which cuda versions are compatible. We have used cuda 11.8.
#### Enter [here](https://developer.nvidia.com/cuda-toolkit-archive) and find the version that you want to install. Select linux, aarch64-jetson, native, Ubuntu and the version. Follow the provided steps to install cuda.
### Install cudNN
#### We have not found any problems with using the latest version of cudNN. Follow [this link](https://developer.nvidia.com/cudnn-downloads) and select your system's specs then follow the steps to install cudNN.
### Install Ultralytics
#### ```pip3 install ultralytics``` should suffice to install ultralytics.
### Install pyTorch
#### ```pip3 install __pipWheelName__``` run this command in the folder where you have downloaded the above mentioned pip wheel.
### Install torchvision
#### Follow [this link](https://forums.developer.nvidia.com/t/pytorch-for-jetson/72048) and follow the steps under the installation section to install the correct torchvision version.
### Verify
#### To verify that cuda, cudNN, torch and torchvision are properly installed try to run the code provided under the verification section from the link above.

### Optional
#### If you are going to use the csi cameras, opencv (the library that handles cameras) needs to be compiled from source.
##### Download [gstreamer](https://gstreamer.freedesktop.org/download/#linux) following the tutorial provided.
##### Download opencv using ```git clone https://github.com/opencv/opencv.git``` and then ```cd opencv```.
##### Run ```cmake -D WITH_GSTREAMER=ON``` in the opencv folder.
#### We have not used gstreamer that much but we managed to make the cameras work. For more details read the camera related code.
#### To verify you can print the result of getBuildInformation method:
```
import cv2

print(cv2.getBuildInformation())
```
#### The string printed should contain: **GSTREAMER YES**

## EDIT
### We have changed the model to yolov10b(ig). It runs okish on Jetson Nano Orin (there are 2 instances of the same model that are running at the same time).
