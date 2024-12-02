from app.field_label.command_line_args import BagCommandLineArgs, CommandLineArgs, TopicCommandLineArgs
from app.field_label.field_label_app import FieldLabelApp
from app.field_label.label_nhrl_cam_config import LabelNhrlCamConfig


class LabelNhrlCam(FieldLabelApp):
    def __init__(self, config: LabelNhrlCamConfig, args: CommandLineArgs) -> None:
        super().__init__(config, args)
