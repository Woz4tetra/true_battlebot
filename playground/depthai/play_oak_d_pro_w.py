import argparse

import depthai as dai
from depthai_sdk import OakCamera


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("path", type=str, help="Path to the recording")
    args = parser.parse_args()

    path = args.path

    with OakCamera(replay=path) as oak:
        color = oak.create_camera("CAM_A")
        # left = oak.create_camera("CAM_B")
        # right = oak.create_camera("CAM_C")

        stereo = oak.create_stereo(resolution="720p")
        stereo.config_stereo(align=color, subpixel=True, lr_check=True)
        stereo.node.setOutputSize(640, 360)  # 720p, downscaled to 640x360 (decimation filter, median filtering)
        # On-device post processing for stereo depth
        config = stereo.node.initialConfig.get()
        stereo.node.setPostProcessingHardwareResources(3, 3)
        config.postProcessing.speckleFilter.enable = True
        config.postProcessing.thresholdFilter.minRange = 400
        config.postProcessing.thresholdFilter.maxRange = 10_000  # 10m
        config.postProcessing.decimationFilter.decimationFactor = 2
        config.postProcessing.decimationFilter.decimationMode = (
            dai.StereoDepthConfig.PostProcessing.DecimationFilter.DecimationMode.NON_ZERO_MEDIAN
        )
        stereo.node.initialConfig.set(config)

        oak.visualize(
            [
                color.out.camera,
                stereo.out.depth,
            ],
            scale=2 / 3,
            fps=True,
        )

        oak.start(blocking=True)


if __name__ == "__main__":
    main()
