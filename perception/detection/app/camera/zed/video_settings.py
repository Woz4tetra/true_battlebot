from __future__ import annotations

from dataclasses import dataclass

import pyzed.sl as sl
from bw_shared.messages.dataclass_utils import from_dict, to_dict


class ZedParameterError(Exception):
    pass


ZED_2I_SETTINGS = (
    sl.VIDEO_SETTINGS.BRIGHTNESS,
    sl.VIDEO_SETTINGS.CONTRAST,
    sl.VIDEO_SETTINGS.HUE,
    sl.VIDEO_SETTINGS.SATURATION,
    sl.VIDEO_SETTINGS.SHARPNESS,
    sl.VIDEO_SETTINGS.GAMMA,
    sl.VIDEO_SETTINGS.GAIN,
    sl.VIDEO_SETTINGS.EXPOSURE,
    sl.VIDEO_SETTINGS.AEC_AGC,
    sl.VIDEO_SETTINGS.AEC_AGC_ROI,
    sl.VIDEO_SETTINGS.WHITEBALANCE_TEMPERATURE,
    sl.VIDEO_SETTINGS.WHITEBALANCE_AUTO,
    sl.VIDEO_SETTINGS.LED_STATUS,
)


@dataclass
class Zed2iVideoSettings:
    # -1 means use the default value, None means do not change the value
    brightness: int | None = None  # Controls image brightness. [0 - 8]
    contrast: int | None = None  # Controls image contrast. [0 - 8]
    hue: int | None = None  # Controls image color. [0 - 11]
    saturation: int | None = None  # Controls image color intensity. [0 - 8]
    sharpness: int | None = None  # Controls image sharpness. [0 - 8]
    gamma: int | None = None  # Controls gamma correction. [0 - 8]
    gain: int | None = None  # Controls digital amplification of the signal from the camera sensors. [0 - 100]
    # Controls shutter speed. Setting a long exposure time leads to an increase in motion blur.
    # [0 - 100] (% of camera frame rate)
    exposure: int | None = None
    aec_agc: int | None = None  # Controls if gain and exposure are in automatic mode or not. [0 - 1]
    # Controls the region of interest for automatic exposure/gain computation. sl::Rect
    aec_agc_roi: tuple[int, int, int, int] | None = None
    whitebalance_temperature: int | None = None  # Controls camera white balance. [2800 - 6500]
    whitebalance_auto: int | None = None  # Controls camera white balance automatic mode. [0 - 1]
    # Controls the status of the camera front LED. Set to 0 to disable the light, 1 to enable the light. [0 - 1]
    led_status: int | None = None

    @classmethod
    def from_camera(cls, camera: sl.Camera) -> Zed2iVideoSettings:
        self = cls()
        for setting in ZED_2I_SETTINGS:
            error_code, value = camera.get_camera_settings(setting)
            if error_code != sl.ERROR_CODE.SUCCESS:
                raise ZedParameterError(f"Failed to get {setting.name}: {error_code}")
            setattr(self, setting.name.lower(), value)
        return self

    def apply_to_camera(self, camera: sl.Camera) -> None:
        for setting in ZED_2I_SETTINGS:
            value = getattr(self, setting.name.lower())
            if value is not None:
                error_code = camera.set_camera_settings(setting, value)
                if error_code != sl.ERROR_CODE.SUCCESS:
                    raise ZedParameterError(f"Failed to set {setting.name} to {value}: {error_code}")

    @classmethod
    def from_dict(cls, data: dict) -> Zed2iVideoSettings:
        return from_dict(cls, data)

    def to_dict(self) -> dict:
        return to_dict(self)
