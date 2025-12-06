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
import tqdm


def main(opt: argparse.Namespace) -> None:
    # Get input parameters
    svo_files = opt.svo_files

    for svo_file in svo_files:
        if not opt.output:
            output_dir = os.path.splitext(svo_file)[0]
        else:
            output_dir = opt.output

        if not os.path.exists(svo_file):
            print("Input SVO file does not exist.\n")
            exit()

        # Create output directories for RGB and depth images
        rgb_dir = os.path.join(output_dir, "rgb")
        depth_dir = os.path.join(output_dir, "depth")
        os.makedirs(rgb_dir, exist_ok=True)
        os.makedirs(depth_dir, exist_ok=True)

        # Specify SVO path parameter
        init_params = sl.InitParameters()
        init_params.depth_mode = sl.DEPTH_MODE.NEURAL_PLUS  # Compute depth for export
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

        # Extract and save camera intrinsics if requested
        if opt.intrinsics_only:
            calibration_params = zed.get_camera_information().camera_configuration.calibration_parameters
            left_cam = calibration_params.left_cam
            fx = left_cam.fx
            fy = left_cam.fy
            cx = left_cam.cx
            cy = left_cam.cy

            # Create 3x3 intrinsic matrix K
            K = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]])

            # Save to file in the specified format
            intrinsics_file = os.path.join(output_dir, "cam_K.txt")
            os.makedirs(output_dir, exist_ok=True)
            np.savetxt(intrinsics_file, K, fmt="%.18e")
            print(f"Camera intrinsics saved to {intrinsics_file}")
            zed.close()
            continue

        # Prepare single image containers
        left_image = sl.Mat()
        depth_image = sl.Mat()

        fps = zed.get_camera_information().camera_configuration.fps
        print(f"SVO frame rate: {fps:.2f} FPS")

        rt_param = sl.RuntimeParameters()

        # Start SVO conversion to images
        print("Extracting RGB and depth images... Use Ctrl-C to interrupt conversion.\n")

        nb_frames = zed.get_svo_number_of_frames()
        pbar = tqdm.tqdm(total=nb_frames)

        frame_count = 0
        while True:
            err = zed.grab(rt_param)
            if err == sl.ERROR_CODE.SUCCESS:
                # Retrieve RGB image (left camera)
                zed.retrieve_image(left_image, sl.VIEW.LEFT)

                # Retrieve depth image
                zed.retrieve_measure(depth_image, sl.MEASURE.DEPTH)

                # Convert RGB image from RGBA to RGB (3 channel, 8-bit)
                rgb_data = left_image.get_data()
                rgb_image = cv2.cvtColor(rgb_data, cv2.COLOR_RGBA2RGB)

                # Get depth data as 16-bit (1 channel)
                # ZED depth is in float32, convert to 16-bit millimeters
                depth_data = depth_image.get_data()
                depth_16bit = np.nan_to_num(depth_data, nan=0.0, posinf=0.0, neginf=0.0)
                depth_16bit = np.clip(depth_16bit, 0, 65535).astype(np.uint16)

                # Save RGB image as JPG
                rgb_filename = os.path.join(rgb_dir, f"frame_{frame_count:06d}.jpg")
                cv2.imwrite(rgb_filename, rgb_image, [cv2.IMWRITE_JPEG_QUALITY, 95])

                # Save depth image as PNG
                depth_filename = os.path.join(depth_dir, f"frame_{frame_count:06d}.png")
                cv2.imwrite(depth_filename, depth_16bit)

                frame_count += 1
                pbar.update(1)

            if err == sl.ERROR_CODE.END_OF_SVOFILE_REACHED:
                sys.stdout.write("\nSVO end has been reached. Exiting now.\n")
                break
        pbar.close()

        print(f"Exported {frame_count} frames to {output_dir}")

        zed.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument("svo_files", type=str, nargs="+", help="Path to the .svo file(s)")
    parser.add_argument(
        "--output",
        type=str,
        help="Path to output directory. If not specified, will create a directory with the same name as the "
        "input file.",
        default="",
    )
    parser.add_argument(
        "--intrinsics-only",
        action="store_true",
        help="Only extract camera intrinsics (K matrix) to cam_K.txt and skip image extraction.",
        default=False,
    )
    opt = parser.parse_args()
    main(opt)
