import cv2 as cv
import numpy

path = "C:\GitHub\SS_Martha\YOLOv8\Training\Images"
file = "buoys_set_1_pic_"
filepath = "{}\{}".format(path, file)

i = 1
startCapture = False

PRESS_TO_TAKE_PICS = True

# Open ZED camera
cap = cv.VideoCapture(0)

# Set the video resolution to HD720 (2560*720)
# Options are: 4416*1242, 3840*1080, 2560*720, 1344*376
# FPS will then be 15,    30/15,     60/30/15, 100/60/30/15 respectively
cap.set(cv.CAP_PROP_FRAME_WIDTH, 2560)
cap.set(cv.CAP_PROP_FRAME_HEIGHT, 720)

if PRESS_TO_TAKE_PICS:
    print("Press 's' to take a picture manually, 'q' to quit")
else:
    print("Press 's' to start automatically take pictures, 'q' to quit")

while cap.isOpened():
    # Get a new frame from camera
    _, frame = cap.read()

    # Extract left and right images from side-by-side
    left_image, right_image = numpy.split(frame, 2, axis=1)

    # Display images
    cv.imshow("left", left_image)

    if PRESS_TO_TAKE_PICS:
        if cv.waitKey(1) == ord('s'):
            cv.imwrite(filepath + "{}.png".format(str(i)), left_image)
            print("Saved to: ", filepath + "{}.png".format(str(i)))
            i += 1
    else:
        if cv.waitKey(1) == ord('s') or startCapture:
            cv.imwrite(filepath + "{}.png".format(str(i)), left_image)
            print("Saved to: ", filepath + "{}.png".format(str(i)))
            startCapture = True
            i += 1

    if cv.waitKey(1) == ord('q'):
        print("Done!")
        break

cap.release()
cv.destroyAllWindows()