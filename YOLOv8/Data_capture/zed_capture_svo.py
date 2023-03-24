########################################################################
#
# Copyright (c) 2022, STEREOLABS.
#
# All rights reserved.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
########################################################################

import pyzed.sl as sl
import cv2 as cv
import keyboard

def main():
    # Create a Camera object
    zed = sl.Camera()

    # Create a InitParameters object and set configuration parameters
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD720  
    init_params.camera_fps = 60  

    # Open the camera
    zedOpened = zed.open(init_params)
    if zedOpened != sl.ERROR_CODE.SUCCESS:
        exit(1)

    print("Camera opened")

    path = "SS_Martha\YOLOv8\Validation"
    file = "buoyvideo.svo"
    filepath = path + file

    # "Bachelor/Maskinsyn/VideoCaptures/test/test.svo"
    recording_param = sl.RecordingParameters(filepath, sl.SVO_COMPRESSION_MODE.H264)
    zedOpened = zed.enable_recording(recording_param)

    startCapture = False
    i = 0
    runtime_parameters = sl.RuntimeParameters()

    print("Press 'q' to quit")
    print("Press 's' to start recording")
    while not keyboard.is_pressed('q'):
        if startCapture or keyboard.is_pressed('s'):
            # Grab an image, a RuntimeParameters object must be given to grab()
            if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
                i = i + 1
                print("Frame count: " + str(i), end="\r")
                startCapture = True
            else:
                print("Runtime error")
                break

    print("Camera closed")
    zed.disable_recording()
    # Close the camera
    zed.close()

if __name__ == "__main__":
    main()