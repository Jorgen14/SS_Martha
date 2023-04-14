from ultralytics import YOLO

# Load a model
model = YOLO('YOLOv8/buoy_detect.pt')  # load a custom model

filepath = "C:\GitHub\SS_Martha\YOLOv8\Test"
filename = "IMG_2088.MOV"

# Validate the model
#metrics = model.predict(source="{}\{}".format(filepath, filename), show=True, save=True, conf=0.5)
metrics = model.predict(source="0", show=True)
