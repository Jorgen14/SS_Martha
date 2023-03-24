from ultralytics import YOLO

# Load a model
model = YOLO('path/to/best.pt')  # load a custom model

# Validate the model
metrics = model.predict(source="", show=True, save=True, conf=0.5)
