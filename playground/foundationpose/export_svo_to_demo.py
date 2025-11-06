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
from pathlib import Path

import cv2
import numpy as np
import pyzed.sl as sl
import tqdm


def progress_bar(percent_done: float, bar_length: int = 50) -> None:
    # Display a progress bar
    done_length = int(bar_length * percent_done / 100)
    bar = "=" * done_length + "-" * (bar_length - done_length)
    sys.stdout.write("[%s] %i%s\r" % (bar, percent_done, "%"))
    sys.stdout.flush()


def export_avi_to_mp4(input_avi: str, output_mp4: str) -> None:
    # use ffmpeg to convert avi to mp4
    command = f"ffmpeg -i {input_avi} -c:v libx264 -crf 23 -preset medium -c:a aac -b:a 192k {output_mp4} -y"
    os.system(command)


def main(opt: argparse.Namespace) -> None:
    # Get input parameters
    svo_files = opt.svo_files

    for svo_file in svo_files:
        if not opt.output:
            output_dir = Path(svo_file).with_suffix()
        else:
            output_dir = Path(opt.output)
        output_dir.mkdir(parents=True, exist_ok=True)

        if not os.path.exists(svo_file):
            print("Input SVO file does not exist.\n")
            exit()

        # Specify SVO path parameter
        init_params = sl.InitParameters()
        init_params.depth_mode = sl.DEPTH_MODE.NEURAL_PLUS  # Compute depth for depth channel export
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

        rt_param = sl.RuntimeParameters()

        # Start SVO conversion to AVI/SEQUENCE
        print("Converting SVO... Use Ctrl-C to interrupt conversion.\n")

        nb_frames = zed.get_svo_number_of_frames()
        pbar = tqdm.tqdm(total=nb_frames)

        while True:
            err = zed.grab(rt_param)
            if err == sl.ERROR_CODE.SUCCESS:
                # Retrieve SVO images
                zed.retrieve_image(image, sl.VIEW.LEFT)
                svo_image_rgba[0:height, 0:width, :] = image.get_data()
                zed.retrieve_image(image, sl.VIEW.DEPTH)
                svo_image_depth = image.get_data()

                # Convert SVO image from RGBA to RGB
                ocv_image_rgb = cv2.cvtColor(svo_image_rgba, cv2.COLOR_RGBA2RGB)

                pbar.update(1)

            if err == sl.ERROR_CODE.END_OF_SVOFILE_REACHED:
                sys.stdout.write("\nSVO end has been reached. Exiting now.\n")
                break
        pbar.close()

        zed.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument("svo_files", type=str, nargs="+", help="Path to the .svo file(s)")
    parser.add_argument(
        "--output",
        type=str,
        help="Path to output svo. If not specified, the output will be saved in the same folder as the input file.",
        default="",
    )
    opt = parser.parse_args()
    main(opt)
