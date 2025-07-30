### [< back](../GuideForDocumentation.md)
# Train AI
## Go to [google colab](https://colab.google/). It is basically a linux machine on the cloud that offers (as of 20.08.2024) 13GB RAM, 15GB GPU RAM and 80GB storage free of charge.
## Go to Edit->Notebook Settings and select Python3 and T4 GPU and press save.
## Make a folder for training your model.
## Drag your archive with the training data (we have used roboflow universe) inside the folder and also make a copy of your pretrained model.
## The folder should contain: data.yaml, model.pt, train(folder), valid(folder), test(folder).
## python3 code is written normally and can be run.
## bash commands must begin with !.
## Install ultralytics using ```!pip3 install ultralytics```

## Use the following command to train your model: 
```!yolo task=detect model=path to yolov10n.pt mode=train data=path to data.yaml batch=32 epochs=100000```
## You can play around with batch and epochs. The model will generally stop training if it does not see improvement from one epoch to another. Batch=32 and epochs=100 are pretty standard values and you should get good results. Use a very big number such as the one in the command for the epochs such that it will stop after getting 90%+ precision/recall.
## The model is found in ```runs/detect/trainX/weights/best.pt```. The location will be printed after training.
## For training "in batches" use: 
```!yolo task=detect model=path to last.pt resume mode=train data=path to data.yaml batch=32 cache=True save=True save_period=10 epochs=100000```
### Note: the command above should be used after training the model for at least one epoch using the first command. Then download the last.pt from the weights folder. The best.pt is the trained model you want to use.

## To test it you can run the following code:
```
from ultralytics import YOLO 
model = YOLO(YOUR MODEL)
print(model.names)
 ```
## It should print all the classes that the model can recognize.

### NOTE: you can train yolo directly on jetson but it is VERY slow. It would have taken ~60 hours to train our yolo. On google colab we spent 7-30 minutes (we have tried different datasets).

## EDIT
### Training yolov10b took ~10 hours to train (same dataset).
### If you are using google colab, make a shared drive (different from just giving someone access to your drive), upload the dataset and the model there and run as described above. The dataset might take a while to upload, try uploading an archive and then just ```!unzip path/to/archive```.
