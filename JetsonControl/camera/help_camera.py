import cv2

# Function that is called when there is a prediction needed on an image without returning it.
def predict(chosen_model, img, classes=[], conf=0.5):
    '''
    Predicts the classes (must be a list) mentioned from the image with the chosen_model with a default confidence of 50%.\n
    Returns the boxes around the detected objects.\n
    '''
    if classes:
        results = chosen_model.predict(img, classes=classes, conf=conf, half=True, stream=True, stream_buffer=False, verbose=False)
    else:
        results = chosen_model.predict(img, conf=conf, verbose=False)
    box_list = []
    for r in results:
        for b in r.boxes:
            box_list.append(b)
    return box_list

# Function that is called when there is a prediction needed on an image + you want to show it in real time thus returning the image.
def predict_and_detect(chosen_model, img, classes=[], conf=0.5, rectangle_thickness=2):
    '''
    Same as predict. Also returns the image.
    '''
    results = predict(chosen_model, img, classes, conf=conf)
    box_list =[]
    for result in results:
        for box in result:
            cv2.rectangle(img, (int(box.xyxy[0][0]), int(box.xyxy[0][1])),
                          (int(box.xyxy[0][2]), int(box.xyxy[0][3])), (255, 0, 0), rectangle_thickness)
            
            box_list.append(box)
    return img, box_list
def track(chosen_model, img, classes=[], conf=0.5, rectangle_thickness=2):
    '''
    Same as predict but track is called instead of predict.\n
    '''
    if classes:
        results = chosen_model.track(img, classes=classes, conf=conf, half=True, stream=True, stream_buffer=False, verbose=False)
    else:
        results = chosen_model.track(img, conf=conf, verbose=False)
    box_list = []
    for r in results:
        for b in r.boxes:
            box_list.append(b)
    return box_list

def track_and_detect(chosen_model, img, classes=[], conf=0.5, rectangle_thickness=2):
    '''
    Same as predict_and_detect but track is called.\n
    '''
    results = track(chosen_model, img, classes, conf=conf)
    box_list =[]
    for result in results:
        for box in result:
            cv2.rectangle(img, (int(box.xyxy[0][0]), int(box.xyxy[0][1])),
                          (int(box.xyxy[0][2]), int(box.xyxy[0][3])), (255, 0, 0), rectangle_thickness)
            
            box_list.append(box)
    return img, box_list
# Function that is called to get the "name" of the CSI camera.
def gstreamer_pipeline(
    sensor_id=0,
    capture_width=1640,
    capture_height=1232,
    display_width=640,
    display_height=480,
    framerate=30,
    flip_method=0,
):
    '''
    Used to get the CSI camera name.
    '''
    return (
        "nvarguscamerasrc sensor-id=%d ! "
        "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            sensor_id,
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height
        )
    ) 
