from ultralytics import YOLO
# Had a problem with this script, something about multiprocessing, 
# so insted the terminal was used to train the model like this:
# cd into the YOLOv8 folder and then run the following command:
# yolo task=detect mode=train epochs=100 data=data.yaml model=yolov8m.pt imgsz=640 

# Load model
model = YOLO('yolov8m.pt') # pass any model type

# Train model
model.train(data="YOLOv8\data.yaml")
