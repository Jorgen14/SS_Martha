from ultralytics import YOLO

# Load model
model = YOLO('yolov8m.pt') # pass any model type

# Train model
model.train(data="YOLOv8\data.yaml")
