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

import sys
import pyzed.sl as sl
import numpy as np
import cv2 as cv


def progress_bar(percent_done, bar_length=50):
    done_length = int(bar_length * percent_done / 100)
    bar = '=' * done_length + '-' * (bar_length - done_length)
    sys.stdout.write('[%s] %f%s\r' % (bar, percent_done, '%'))
    sys.stdout.flush()


def main():
    # Get input parameters
    filepath = "C:\GitHub\SS_Martha\YOLOv8\Validation"
    filename = "buoyvideo_1"
    inputFileName = "{}.svo".format(filename)

    outputLeft = "{}_left".format(filename)
    outputRight = "{}_right".format(filename)
    outputDepth = "{}_depth".format(filename)
    newVideoFileLeft = "{}.avi".format(outputLeft)
    newVideoFileRight = "{}.avi".format(outputRight)
    newVideoFileDepth = "{}.avi".format(outputDepth)

    svo_input_path = "{0}\{1}".format(filepath, inputFileName)
    output_path_left = "{0}\{1}".format(filepath, newVideoFileLeft)
    output_path_right = "{0}\{1}".format(filepath, newVideoFileRight)
    output_path_depth = "{0}\{1}".format(filepath, newVideoFileDepth)
    output_as_video = True    

    # Specify SVO path parameter
    init_params = sl.InitParameters()
    init_params.set_from_svo_file(str(svo_input_path))
    init_params.svo_real_time_mode = False  # Don't convert in realtime
    init_params.coordinate_units = sl.UNIT.MILLIMETER  # Use milliliter units (for depth measurements)

    # Create ZED objects
    zed = sl.Camera()

    # Open the SVO file specified as a parameter
    err = zed.open(init_params)
    if err != sl.ERROR_CODE.SUCCESS:
        sys.stdout.write(repr(err))
        zed.close()
        exit()
    
    # Get image size
    image_size = zed.get_camera_information().camera_resolution
    width = image_size.width
    height = image_size.height

    # Prepare single image containers
    left_image = sl.Mat()
    right_image = sl.Mat()
    depth_image = sl.Mat()

    svo_image_left_rgba = np.zeros((height, width, 4), dtype=np.uint8)
    svo_image_right_rgba = np.zeros((height, width, 4), dtype=np.uint8)
    svo_image_depth_rgba = np.zeros((height, width, 4), dtype=np.uint8)

    video_writer_left = None
    video_writer_right = None
    video_writer_depth = None
    if output_as_video:
        # Create video writer with MPEG-4 part 2 codec
        video_writer_left = cv.VideoWriter(str(output_path_left),
                                       cv.VideoWriter_fourcc('M', '4', 'S', '2'),
                                       max(zed.get_camera_information().camera_fps, 25),
                                       (width, height))
        
        video_writer_right = cv.VideoWriter(str(output_path_right),
                                       cv.VideoWriter_fourcc('M', '4', 'S', '2'),
                                       max(zed.get_camera_information().camera_fps, 25),
                                       (width, height))
        
        video_writer_depth = cv.VideoWriter(str(output_path_depth),
                                       cv.VideoWriter_fourcc('M', '4', 'S', '2'),
                                       max(zed.get_camera_information().camera_fps, 25),
                                       (width, height))

        if not video_writer_left.isOpened() or not video_writer_right.isOpened() or not video_writer_depth.isOpened():
            sys.stdout.write("OpenCV video writer cannot be opened. Please check the .avi file path and write "
                             "permissions.\n")
            zed.close()
            exit()
    
    rt_param = sl.RuntimeParameters()
    rt_param.sensing_mode = sl.SENSING_MODE.FILL

    # Start SVO conversion to AVI/SEQUENCE
    sys.stdout.write("Converting SVO... Use Ctrl-C to interrupt conversion.\n")

    nb_frames = zed.get_svo_number_of_frames()

    while True:
        if zed.grab(rt_param) == sl.ERROR_CODE.SUCCESS:
            svo_position = zed.get_svo_position()

            # Retrieve SVO images
            zed.retrieve_image(left_image, sl.VIEW.LEFT)
            zed.retrieve_image(right_image, sl.VIEW.RIGHT)
            zed.retrieve_image(depth_image, sl.VIEW.DEPTH)
            #zed.retrieve_measure(depth_image, sl.MEASURE.DEPTH)

            if output_as_video:
                svo_image_left_rgba[0:height, 0:width, :] = left_image.get_data()
                # Convert SVO image from RGBA to RGB
                ocv_image_left_rgb = cv.cvtColor(svo_image_left_rgba, cv.COLOR_RGBA2RGB)

                svo_image_right_rgba[0:height, 0:width, :] = right_image.get_data()
                # Convert SVO image from RGBA to RGB
                ocv_image_right_rgb = cv.cvtColor(svo_image_right_rgba, cv.COLOR_RGBA2RGB)

                svo_image_depth_rgba[0:height, 0:width, :] = depth_image.get_data()
                # Convert SVO image from RGBA to RGB
                ocv_image_depth_rgb = cv.cvtColor(svo_image_depth_rgba, cv.COLOR_RGBA2RGB)

                # Write the RGB image in the video
                video_writer_left.write(ocv_image_left_rgb)
                video_writer_right.write(ocv_image_right_rgb)
                video_writer_depth.write(ocv_image_depth_rgb)
            else:
                print("Nokka e riv ruskanes galt!")

            # Display progress
            progress_bar((svo_position + 1) / nb_frames * 100, 30)

            # Check if we have reached the end of the video
            if svo_position >= (nb_frames - 1):  # End of SVO
                sys.stdout.write("\nSVO end has been reached. Exiting now.\n")
                break

    if output_as_video:
        # Close the video writer
        video_writer_left.release()

    print("Resolution: {0}, {1}.".format(round(zed.get_camera_information().camera_resolution.width, 2), zed.get_camera_information().camera_resolution.height))
    print("Camera FPS: {0}".format(zed.get_camera_information().camera_fps))
    print("Frame count: {0}.\n".format(zed.get_svo_number_of_frames()))

    zed.close()
    return 0


if __name__ == "__main__":
    main()