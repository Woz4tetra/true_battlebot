from pprint import pprint

from detectron2 import model_zoo
from detectron2.config import CfgNode, get_cfg


def recurse_config(cfg: CfgNode, data: dict) -> None:
    for key in cfg.keys():
        value = cfg[key]
        if isinstance(value, CfgNode):
            data[key] = {}
            recurse_config(value, data[key])
        else:
            data[key] = value


def main() -> None:
    config_file_path = "COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x.yaml"
    cfg = get_cfg()
    cfg.merge_from_file(model_zoo.get_config_file(config_file_path))
    data = {}
    recurse_config(cfg, data=data)
    pprint(data)


if __name__ == "__main__":
    main()
