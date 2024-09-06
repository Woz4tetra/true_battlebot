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

import argparse
import os
import sys

import cv2
import numpy as np
import pyzed.sl as sl


def progress_bar(percent_done, bar_length=50):
    # Display a progress bar
    done_length = int(bar_length * percent_done / 100)
    bar = "=" * done_length + "-" * (bar_length - done_length)
    sys.stdout.write("[%s] %i%s\r" % (bar, percent_done, "%"))
    sys.stdout.flush()


def main(opt: argparse.Namespace):
    # Get input parameters
    svo_files = opt.svo_files
    channel = opt.channel

    for svo_file in svo_files:
        if not opt.output:
            output = os.path.splitext(svo_file)[0] + ".avi"
        else:
            output = opt.output

        if not os.path.exists(svo_file):
            print("Input SVO file does not exist.\n")
            exit()

        # Specify SVO path parameter
        init_params = sl.InitParameters()
        init_params.set_from_svo_file(svo_file)
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
        image_size = zed.get_camera_information().camera_configuration.resolution
        width = image_size.width
        height = image_size.height

        # Prepare side by side image container equivalent to CV_8UC4
        svo_image_rgba = np.zeros((height, width, 4), dtype=np.uint8)

        # Prepare single image containers
        image = sl.Mat()

        # Create video writer with MPEG-4 part 2 codec
        video_writer = cv2.VideoWriter(
            output,
            cv2.VideoWriter_fourcc("M", "4", "S", "2"),  # type: ignore
            max(zed.get_camera_information().camera_configuration.fps, 100),
            (width, height),
        )
        if not video_writer.isOpened():
            print("OpenCV video writer cannot be opened. Please check the .avi file path and write permissions.")
            zed.close()
            exit()

        rt_param = sl.RuntimeParameters()

        # Start SVO conversion to AVI/SEQUENCE
        print("Converting SVO... Use Ctrl-C to interrupt conversion.\n")

        nb_frames = zed.get_svo_number_of_frames()

        while True:
            err = zed.grab(rt_param)
            if err == sl.ERROR_CODE.SUCCESS:
                svo_position = zed.get_svo_position()

                # Retrieve SVO images
                if channel == 0:
                    zed.retrieve_image(image, sl.VIEW.LEFT)
                elif channel == 1:
                    zed.retrieve_image(image, sl.VIEW.RIGHT)
                elif channel == 2:
                    zed.retrieve_image(image, sl.VIEW.DEPTH)

                # Copy the left image to the left side of SBS image
                svo_image_rgba[0:height, 0:width, :] = image.get_data()

                # Convert SVO image from RGBA to RGB
                ocv_image_rgb = cv2.cvtColor(svo_image_rgba, cv2.COLOR_RGBA2RGB)

                # Write the RGB image in the video
                video_writer.write(ocv_image_rgb)

                # Display progress
                progress_bar((svo_position + 1) / nb_frames * 100, 30)
            if err == sl.ERROR_CODE.END_OF_SVOFILE_REACHED:
                progress_bar(100, 30)
                sys.stdout.write("\nSVO end has been reached. Exiting now.\n")
                break

        # Close the video writer
        video_writer.release()

        zed.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument("svo_files", type=str, nargs="+", help="Path to the .svo file(s)")
    parser.add_argument("--channel", type=int, help="Channel to export (0: Left, 1: Right, 3: Depth)", default=0)
    parser.add_argument(
        "--output",
        type=str,
        help="Path to output svo. If not specified, the output will be saved in the same folder as the input file.",
        default="",
    )
    opt = parser.parse_args()
    main(opt)
