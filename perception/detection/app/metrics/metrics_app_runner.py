from pathlib import Path

from app.config.metrics_tool.load_config import load_config
from app.container import Container
from app.metrics.command_line_args import CommandLineArgs
from app.metrics.metrics_app import MetricsApp
from app.metrics.tracker.tracker_loader import load_tracker
from bw_shared.configs.shared_config import SharedConfig


def run_app(args: CommandLineArgs) -> None:
    container = Container()

    config_path = Path(args.config)
    video_path = Path(args.video_file)
    config = load_config(config_path)
    shared_config = SharedConfig.from_files()
    map_config = shared_config.get_map(config.field.type)

    container.register(config)
    container.register(shared_config)

    tracker = load_tracker(container, config.tracker)

    MetricsApp(video_path, config_path, config, map_config, tracker).run()
