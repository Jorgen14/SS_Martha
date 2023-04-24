from ultralytics import YOLO
from datetime import datetime
import cv2

# Load a model
model = YOLO('YOLOv8/buoy_s.pt')  # load a custom model

filepath = "C:\GitHub\SS_Martha\YOLOv8\Test"
#filename = "IMG_2088.MOV" 
filename = "buoys_set_1_vid.mp4"  

# # Validate the model
# results = model.predict(source="{}\{}".format(filepath, filename), show=True, save=False, conf=0.5)
# #results = model.predict(source="0", show=True)
# for r in results:
#     for c in r.boxes.cls:
#         print(model.names[int(c)])


cam = cv2.VideoCapture("{}\{}".format(filepath, filename))
itt = 0

while 1:
    startTime = datetime.now()
    ret, frame = cam.read()
    if ret:#itt == 0:
        # Validate the model
        results = model.predict(source=frame, show=True, save=False, conf=0.2)
        det_center = []
        det_names = []
        for result in results:
            for box in result.boxes.xyxy:
                center = ((box[2].item() - box[0].item()) / 2 + box[0].item(), 
                          (box[3].item() - box[1].item()) / 2 + box[1].item())
                det_center.append(center)
            for d_cls in result.boxes.cls:
                det_names.append(model.names[int(d_cls)])
        print("Center of detections: ", det_center)
        print("Names of detections: ", det_names)
        #itt += 1
        print("Time to process: ", datetime.now() - startTime)
    else:
        break

cam.release()
cv2.destroyAllWindows()