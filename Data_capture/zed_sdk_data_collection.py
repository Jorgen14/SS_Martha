import cv2 as cv
import pyzed.sl as sl

path = "/home/navo/GitHub/SS_Martha/Videos" 
file = "buoys_video_set_1"
filepath = "{}/{}".format(path, file)

zed = sl.Camera()
init_params = sl.InitParameters()
init_params.camera_resolution = sl.RESOLUTION.HD2K # Options: HD2K, HD1080, HD720, VGA

runtime_params = sl.RuntimeParameters()
zed_status = zed.open(init_params)

zed.set_camera_settings(sl.VIDEO_SETTINGS.EXPOSURE, -1)
print("Exposure: ", zed.get_camera_settings(sl.VIDEO_SETTINGS.EXPOSURE))

img_left = sl.Mat()

fourcc = cv.VideoWriter_fourcc(*'mp4v')
#out = cv.VideoWriter("{}.mp4".format(filepath), fourcc, 30.0, (1280, 720))
#out = cv.VideoWriter("{}.mp4".format(filepath), fourcc, 15.0, (1920, 1080))
out = cv.VideoWriter("{}.mp4".format(filepath), fourcc, 15.0, (2208, 1242))

while zed.grab(runtime_params) == sl.ERROR_CODE.SUCCESS:
    zed.retrieve_image(img_left, sl.VIEW.LEFT)
    np_img_left = img_left.get_data()
    np_img_left = np_img_left[:,:,:3] # Remove alpha channel
    out.write(np_img_left)
else:
    print("Error grabbing image")

print("Written to: ", filepath + ".mp4")

zed.close()
cv.destroyAllWindows()
