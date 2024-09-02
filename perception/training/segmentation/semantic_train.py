import argparse
import warnings
from pathlib import Path

import cv2
import matplotlib
import numpy as np
import torch
import torch.nn as nn
from livelossplot import PlotLosses
from livelossplot.outputs.matplotlib_plot import MatplotlibPlot
from perception_tools.config.model_metadata import FIELD_SEMANTIC_MODEL_METADATA
from perception_tools.inference.common import get_default_device
from perception_tools.inference.deeplabv3 import IMAGE_SIZE, PAD_SIZE, common_transforms
from perception_tools.training.deeplabv3 import seed_everything
from torch.nn import functional
from torch.utils.data import DataLoader, Dataset
from torchmetrics import MeanMetric
from torchvision.models.segmentation import deeplabv3_mobilenet_v3_large, deeplabv3_resnet50, deeplabv3_resnet101
from tqdm import tqdm

matplotlib.use("agg")


class SegDataset(Dataset):
    def __init__(
        self, *, img_paths: list[Path], mask_paths: list[Path], image_size=(IMAGE_SIZE, IMAGE_SIZE), data_type="train"
    ):
        self.data_type = data_type
        self.img_paths = img_paths
        self.mask_paths = mask_paths
        self.image_size = image_size

        self.transforms = common_transforms()

    def read_file(self, path: Path):
        image = cv2.imread(str(path))
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        image = cv2.resize(image, self.image_size, interpolation=cv2.INTER_NEAREST)
        return image

    def __len__(self):
        return len(self.img_paths)

    def __getitem__(self, index: int):
        image_path = self.img_paths[index]
        image = self.read_file(image_path)
        image = self.transforms(image)

        mask_path = self.mask_paths[index]

        gt_mask = self.read_file(mask_path).astype(np.int32)

        mask = np.zeros((*self.image_size, 2), dtype=np.float32)

        # BACKGROUND
        mask[:, :, 0] = np.where(gt_mask[:, :, 0] == 0, 1.0, 0.0)
        # FIELD
        mask[:, :, 1] = np.where(gt_mask[:, :, 0] > 0, 1.0, 0.0)

        mask = np.pad(mask, ((PAD_SIZE, PAD_SIZE), (PAD_SIZE, PAD_SIZE), (0, 0)), mode="constant", constant_values=0)

        reordered_mask = torch.from_numpy(mask).permute(2, 0, 1)

        return image, reordered_mask


def get_training_set_paths(data_directory: Path) -> tuple[list[Path], list[Path]]:
    image_paths = []
    for path in data_directory.iterdir():
        if path.suffix == ".jpg":
            image_paths.append(path)
    filtered_image_paths = []
    annotation_paths = []
    for path in image_paths:
        anno_name = Path(str(path).replace(".jpg", "_mask.png"))
        if not anno_name.exists():
            warnings.warn(f"Missing mask for image {path}")
            continue
        annotation_paths.append(anno_name)
        filtered_image_paths.append(path)
    return filtered_image_paths, annotation_paths


def get_dataset(data_directory: Path, batch_size: int, num_workers: int):
    train_img_dir = data_directory / "train"
    valid_img_dir = data_directory / "val"

    train_img_paths, train_msk_paths = get_training_set_paths(train_img_dir)
    valid_img_paths, valid_msk_paths = get_training_set_paths(valid_img_dir)

    train_ds = SegDataset(img_paths=train_img_paths, mask_paths=train_msk_paths, data_type="train")
    valid_ds = SegDataset(img_paths=valid_img_paths, mask_paths=valid_msk_paths, data_type="valid")

    train_loader = DataLoader(train_ds, batch_size=batch_size, num_workers=num_workers, shuffle=True, pin_memory=True)
    valid_loader = DataLoader(valid_ds, batch_size=batch_size, num_workers=num_workers, shuffle=False, pin_memory=True)

    return train_loader, valid_loader


def prepare_model(backbone_model: str, num_classes: int):
    # Initialize model with pre-trained weights.
    weights = "DEFAULT"
    model = {
        "mbv3": deeplabv3_mobilenet_v3_large,
        "r50": deeplabv3_resnet50,
        "r101": deeplabv3_resnet101,
    }[backbone_model](weights=weights)

    # Update the number of output channels for the output layer.
    # This will remove the pre-trained weights for the last layer.
    model.classifier[4] = nn.LazyConv2d(num_classes, 1)  # type: ignore
    model.aux_classifier[4] = nn.LazyConv2d(num_classes, 1)  # type: ignore
    return model


def intermediate_metric_calculation(predictions, targets, use_dice=False, smooth=1e-6, dims=(2, 3)):
    # dimscorresponding to image height and width: [B, C, H, W].

    # Intersection: |G ∩ P|. Shape: (batch_size, num_classes)
    intersection = (predictions * targets).sum(dim=dims) + smooth

    # Summation: |G| + |P|. Shape: (batch_size, num_classes).
    summation = (predictions.sum(dim=dims) + targets.sum(dim=dims)) + smooth

    if use_dice:
        # Dice Shape: (batch_size, num_classes)
        metric = (2.0 * intersection) / summation
    else:
        # Union. Shape: (batch_size, num_classes)
        union = summation - intersection

        # IoU Shape: (batch_size, num_classes)
        metric = intersection / union

    # Compute the mean over the remaining axes (batch and classes).
    # Shape: Scalar
    total = metric.mean()

    return total


class Loss(nn.Module):
    def __init__(self, smooth=1e-6, use_dice=False):
        super().__init__()
        self.smooth = smooth
        self.use_dice = use_dice

    def forward(self, predictions, targets):
        # predictions --> (B, #C, H, W) unnormalized
        # targets     --> (B, #C, H, W) one-hot encoded

        # Normalize model predictions
        predictions = torch.sigmoid(predictions)

        # Calculate pixel-wise loss for both channels. Shape: Scalar
        pixel_loss = functional.binary_cross_entropy(predictions, targets, reduction="mean")

        mask_loss = 1 - intermediate_metric_calculation(
            predictions, targets, use_dice=self.use_dice, smooth=self.smooth
        )
        total_loss = mask_loss + pixel_loss

        return total_loss


def convert_2_onehot(matrix, num_classes=3):
    """
    Perform one-hot encoding across the channel dimension.
    """
    matrix = matrix.permute(0, 2, 3, 1)
    matrix = torch.argmax(matrix, dim=-1)
    matrix = functional.one_hot(matrix, num_classes=num_classes)
    matrix = matrix.permute(0, 3, 1, 2)

    return matrix


class Metric(nn.Module):
    def __init__(self, num_classes=3, smooth=1e-6, use_dice=False):
        super().__init__()
        self.num_classes = num_classes
        self.smooth = smooth
        self.use_dice = use_dice

    def forward(self, predictions, targets):
        # predictions  --> (B, #C, H, W) unnormalized
        # targets      --> (B, #C, H, W) one-hot encoded

        # Converting unnormalized predictions into one-hot encoded across channels.
        # Shape: (B, #C, H, W)
        predictions = convert_2_onehot(predictions, num_classes=self.num_classes)  # one hot encoded

        metric = intermediate_metric_calculation(predictions, targets, use_dice=self.use_dice, smooth=self.smooth)

        # Compute the mean over the remaining axes (batch and classes). Shape: Scalar
        return metric


def to_device(data, device):
    """Move tensor(s) to chosen device"""
    if isinstance(data, (list, tuple)):
        return [to_device(x, device) for x in data]
    return data.to(device, non_blocking=True)


class DeviceDataLoader:
    """Wrap a dataloader to move data to a device"""

    def __init__(self, dl, device):
        self.dl = dl
        self.device = device

    def __iter__(self):
        """Yield a batch of data after moving it to device"""
        for b in self.dl:
            yield to_device(b, self.device)

    def __len__(self):
        """Number of batches"""
        return len(self.dl)


class TrainingStepInterface:
    def __init__(self, model) -> None:
        self.model = model

    def step(self, data) -> tuple[torch.Tensor, torch.Tensor]:
        raise NotImplementedError


class TrainStep(TrainingStepInterface):
    def __init__(self, model, optimizer_fn, loss_fn) -> None:
        super().__init__(model)
        self.optimizer_fn = optimizer_fn
        self.loss_fn = loss_fn

    def step(self, data) -> tuple[torch.Tensor, torch.Tensor]:
        preds = self.model(data[0])["out"]

        loss = self.loss_fn(preds, data[1])

        self.optimizer_fn.zero_grad()
        loss.backward()
        self.optimizer_fn.step()
        return preds, loss


class ValidationStep(TrainingStepInterface):
    def __init__(self, model, loss_fn) -> None:
        super().__init__(model)
        self.loss_fn = loss_fn

    def step(self, data) -> tuple[torch.Tensor, torch.Tensor]:
        with torch.no_grad():
            preds = self.model(data[0])["out"].detach()

        loss = self.loss_fn(preds, data[1])

        return preds, loss


def step(epoch_num: int, loader: DeviceDataLoader, step_interface: TrainingStepInterface, metric_fn, step_name):
    loss_record = MeanMetric()
    metric_record = MeanMetric()

    loader_len = len(loader)

    for data in tqdm(iterable=loader, total=loader_len, dynamic_ncols=True, desc=f"{step_name} :: Epoch: {epoch_num}"):
        preds, loss = step_interface.step(data)

        metric = metric_fn(preds.detach(), data[1])

        loss_value = loss.detach().item()
        metric_value = metric.detach().item()

        loss_record.update(loss_value)
        metric_record.update(metric_value)

    current_loss = loss_record.compute()
    current_metric = metric_record.compute()

    return current_loss, current_metric


def main() -> None:
    # For reproducibility
    seed = 4176
    seed_everything(seed)

    parser = argparse.ArgumentParser(description="Train a semantic segmentation model")
    parser.add_argument(
        "dataset_location",
        type=str,
        help="Path to the directory containing the images and annotations",
    )
    parser.add_argument(
        "-c",
        "--checkpoint",
        type=str,
        default=None,
        help="Path to the checkpoint file",
    )
    parser.add_argument(
        "-b",
        "--batch-size",
        type=int,
        default=8,
        help="Batch size",
    )
    parser.add_argument(
        "-nw",
        "--num-workers",
        type=int,
        default=4,
        help="Number of workers",
    )
    parser.add_argument(
        "-ne",
        "--num-epochs",
        type=int,
        default=500,
        help="Number of training epochs",
    )
    parser.add_argument(
        "-o",
        "--output",
        type=str,
        default="",
        help="Output directory for the model and logs",
    )
    assert torch.cuda.is_available(), "CUDA is not available"
    assert torch.cuda.device_count() > 0, "No CUDA devices available"

    args = parser.parse_args()
    dataset_location = Path(args.dataset_location)
    checkpoint_path = Path(args.checkpoint) if args.checkpoint else None
    output = Path(args.output) if args.output else dataset_location.parent / "output"
    batch_size = args.batch_size
    num_workers = args.num_workers
    num_epochs = args.num_epochs
    num_classes = len(FIELD_SEMANTIC_MODEL_METADATA.labels)
    backbone_model_name = "r50"  # mbv3 | r50 | r101

    output.mkdir(parents=True, exist_ok=True)

    output_model = output / f"model_{backbone_model_name}.pth"
    fig_path = output / "plot.png"

    device = get_default_device()

    model = prepare_model(backbone_model=backbone_model_name, num_classes=num_classes)
    model.to(device)
    if checkpoint_path is not None:
        checkpoint_model_name = checkpoint_path.stem.split("_")[1]
        assert (
            checkpoint_model_name == backbone_model_name
        ), f"Model backbone mismatch. {checkpoint_model_name} != {backbone_model_name}"
        checkpoints = torch.load(checkpoint_path, map_location=device)
        model.load_state_dict(checkpoints, strict=False)
        model.eval()

    # Dummy pass through the model
    _ = model(torch.randn((2, 3, IMAGE_SIZE + 2 * PAD_SIZE, IMAGE_SIZE + 2 * PAD_SIZE), device=device))

    train_loader, valid_loader = get_dataset(
        data_directory=dataset_location, batch_size=batch_size, num_workers=num_workers
    )
    for i, j in valid_loader:
        print(f"Image shape: {i.shape}, Image type: {i.dtype}, Mask shape: {j.shape}, Mask type: {j.dtype}")
        break
    train_device_loader = DeviceDataLoader(train_loader, device)
    valid_device_loader = DeviceDataLoader(valid_loader, device)

    metric_name = "iou"
    use_dice = True if metric_name == "dice" else False

    metric_fn = Metric(num_classes=num_classes, use_dice=use_dice).to(device)
    loss_fn = Loss(use_dice=use_dice).to(device)

    optimizer = torch.optim.Adam(model.parameters(), lr=0.0001)

    liveloss = PlotLosses(
        outputs=[MatplotlibPlot(figpath=str(fig_path)), "ExtremaPrinter"],  # type: ignore
        mode="script",
    )

    best_metric = 0.0
    train_step = TrainStep(model, optimizer, loss_fn)
    valid_step = ValidationStep(model, loss_fn)

    for epoch in range(1, num_epochs + 1):
        logs = {}

        model.train()
        train_loss, train_metric = step(
            epoch_num=epoch,
            loader=train_device_loader,
            step_interface=train_step,
            metric_fn=metric_fn,
            step_name="train",
        )

        model.eval()
        valid_loss, valid_metric = step(
            epoch_num=epoch,
            loader=valid_device_loader,
            step_interface=valid_step,
            metric_fn=metric_fn,
            step_name="valid",
        )

        logs["loss"] = train_loss
        logs[metric_name] = train_metric
        logs["val_loss"] = valid_loss
        logs[f"val_{metric_name}"] = valid_metric

        liveloss.update(logs)
        liveloss.send()

        if valid_metric >= best_metric:
            print(f"Saving model. {valid_metric:0.4f} >= {best_metric:0.4f}")
            torch.save(model.state_dict(), output_model)
            best_metric = valid_metric


if __name__ == "__main__":
    main()
