import argparse
import os
import warnings
from pathlib import Path

import numpy as np
import pytorch_lightning as pl
import torch
from bw_shared.enums.label import Label
from datasets import load_metric
from perception_tools.config.model_metadata import LABEL_COLORS
from PIL import Image
from pytorch_lightning.callbacks.early_stopping import EarlyStopping
from pytorch_lightning.callbacks.model_checkpoint import ModelCheckpoint
from torch import nn
from torch.utils.data import DataLoader, Dataset
from transformers import SegformerForSemanticSegmentation, SegformerImageProcessor


class SemanticSegmentationDataset(Dataset):
    """Image (semantic) segmentation dataset."""

    def __init__(self, root_dir, image_processor, train=True):
        """
        Args:
            root_dir (string): Root directory of the dataset containing the images + annotations.
            image_processor (SegFormerImageProcessor): image processor to prepare images + segmentation maps.
            train (bool): Whether to load "training" or "validation" images + annotations.
        """
        self.root_dir = root_dir
        self.image_processor = image_processor
        self.train = train

        self.classes_csv_file = os.path.join(self.root_dir, "_classes.csv")
        with open(self.classes_csv_file, "r") as fid:
            data = [label.split(",") for index, label in enumerate(fid) if index != 0]
        self.id2label = {x[0]: x[1] for x in data}

        # read images and annotations
        image_file_names = [f for f in os.listdir(self.root_dir) if ".jpg" in f]
        self.annotations = []

        self.images = []
        for image_name in sorted(image_file_names):
            anno_name = image_name.replace(".jpg", "_mask.png")
            if not os.path.isfile(os.path.join(self.root_dir, anno_name)):
                warnings.warn(f"Missing mask for image {image_name}")
                continue
            self.annotations.append(anno_name)
            self.images.append(image_name)

    def __len__(self):
        return len(self.images)

    def __getitem__(self, idx):
        image = Image.open(os.path.join(self.root_dir, self.images[idx]))
        segmentation_map = Image.open(os.path.join(self.root_dir, self.annotations[idx]))

        # randomly crop + pad both image and segmentation map to same size
        encoded_inputs = self.image_processor(image, segmentation_map, return_tensors="pt")

        for k, v in encoded_inputs.items():
            encoded_inputs[k].squeeze_()  # remove batch dimension

        return encoded_inputs


class SegformerFinetuner(pl.LightningModule):
    def __init__(
        self,
        pretrained_model_name_or_path,
        id2label,
        train_dataloader=None,
        val_dataloader=None,
        test_dataloader=None,
        metrics_interval=100,
        log_dir="lightning_logs",
    ):
        super(SegformerFinetuner, self).__init__()
        self.id2label = id2label
        self.metrics_interval = metrics_interval
        self.train_dl = train_dataloader
        self.val_dl = val_dataloader
        self.test_dl = test_dataloader

        self.num_classes = len(id2label.keys())
        self.label2id = {v: k for k, v in self.id2label.items()}

        self.model = SegformerForSemanticSegmentation.from_pretrained(
            pretrained_model_name_or_path,
            return_dict=False,
            num_labels=self.num_classes,
            id2label=self.id2label,
            label2id=self.label2id,
            ignore_mismatched_sizes=True,
        )

        self.train_mean_iou = load_metric("mean_iou")
        self.val_mean_iou = load_metric("mean_iou")
        self.test_mean_iou = load_metric("mean_iou")

    def forward(self, images, masks):
        outputs = self.model(pixel_values=images, labels=masks)
        return outputs

    def training_step(self, batch, batch_nb):
        images, masks = batch["pixel_values"], batch["labels"]

        outputs = self(images, masks)

        loss, logits = outputs[0], outputs[1]

        upsampled_logits = nn.functional.interpolate(
            logits, size=masks.shape[-2:], mode="bilinear", align_corners=False
        )

        predicted = upsampled_logits.argmax(dim=1)

        self.train_mean_iou.add_batch(
            predictions=predicted.detach().cpu().numpy(), references=masks.detach().cpu().numpy()
        )
        if batch_nb % self.metrics_interval == 0:
            metrics = self.train_mean_iou.compute(
                num_labels=self.num_classes,
                ignore_index=255,
                reduce_labels=False,
            )

            metrics = {"loss": loss, "mean_iou": metrics["mean_iou"], "mean_accuracy": metrics["mean_accuracy"]}

            for k, v in metrics.items():
                self.log(k, v)

            return metrics
        else:
            return {"loss": loss}

    def validation_step(self, batch, batch_nb):
        images, masks = batch["pixel_values"], batch["labels"]

        outputs = self(images, masks)

        loss, logits = outputs[0], outputs[1]

        upsampled_logits = nn.functional.interpolate(
            logits, size=masks.shape[-2:], mode="bilinear", align_corners=False
        )

        predicted = upsampled_logits.argmax(dim=1)

        self.val_mean_iou.add_batch(
            predictions=predicted.detach().cpu().numpy(), references=masks.detach().cpu().numpy()
        )

        self.log("val_loss", loss)
        return {"val_loss": loss}

    def test_step(self, batch, batch_nb):
        images, masks = batch["pixel_values"], batch["labels"]

        outputs = self(images, masks)

        loss, logits = outputs[0], outputs[1]

        upsampled_logits = nn.functional.interpolate(
            logits, size=masks.shape[-2:], mode="bilinear", align_corners=False
        )

        predicted = upsampled_logits.argmax(dim=1)

        self.test_mean_iou.add_batch(
            predictions=predicted.detach().cpu().numpy(), references=masks.detach().cpu().numpy()
        )
        self.log("test_loss", loss)

        return {"test_loss": loss}

    def configure_optimizers(self):
        return torch.optim.Adam([p for p in self.parameters() if p.requires_grad], lr=2e-05, eps=1e-08)

    def train_dataloader(self):
        return self.train_dl

    def val_dataloader(self):
        return self.val_dl

    def test_dataloader(self):
        return self.test_dl


color_map = {
    0: (0, 0, 0),
    1: LABEL_COLORS[Label.FIELD].to_cv_color(),
}


def prediction_to_vis(prediction):
    vis_shape = prediction.shape + (3,)
    vis = np.zeros(vis_shape)
    for i, c in color_map.items():
        vis[prediction == i] = color_map[i]
    return Image.fromarray(vis.astype(np.uint8))


def main() -> None:
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
        default=4,
        help="Batch size",
    )
    parser.add_argument(
        "-nw",
        "--num-workers",
        type=int,
        default=2,
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
    args = parser.parse_args()
    dataset_location = Path(args.dataset_location)
    checkpoint = args.checkpoint
    output = Path(args.output) if args.output else dataset_location.parent / "output"

    pretrained_model_name_or_path = "nvidia/segformer-b5-finetuned-ade-640-640"

    image_processor = SegformerImageProcessor(reduce_labels=True)

    train_dataset = SemanticSegmentationDataset(str(dataset_location / "train/"), image_processor)
    val_dataset = SemanticSegmentationDataset(str(dataset_location / "val/"), image_processor, train=False)
    test_dataset = SemanticSegmentationDataset(str(dataset_location / "test/"), image_processor, train=False)

    batch_size = args.batch_size
    num_workers = args.num_workers
    train_dataloader = DataLoader(train_dataset, batch_size=batch_size, shuffle=True, num_workers=num_workers)
    val_dataloader = DataLoader(val_dataset, batch_size=batch_size, num_workers=num_workers)
    test_dataloader = DataLoader(test_dataset, batch_size=batch_size, num_workers=num_workers)

    segformer_finetuner = SegformerFinetuner(
        pretrained_model_name_or_path,
        train_dataset.id2label,
        train_dataloader=train_dataloader,
        val_dataloader=val_dataloader,
        test_dataloader=test_dataloader,
        metrics_interval=10,
    )
    early_stop_callback = EarlyStopping(
        monitor="val_loss",
        min_delta=0.00,
        patience=10,
        verbose=False,
        mode="min",
    )

    checkpoint_callback = ModelCheckpoint(save_top_k=1, monitor="val_loss")

    trainer = pl.Trainer(
        callbacks=[early_stop_callback, checkpoint_callback],
        max_epochs=args.num_epochs,
        val_check_interval=1.0,
    )
    trainer.fit(segformer_finetuner, ckpt_path=checkpoint)

    segformer_finetuner.model.save_pretrained(output)
    print(f"Saved model to {output}")

    res = trainer.test(segformer_finetuner, ckpt_path=checkpoint)
    print(res)


if __name__ == "__main__":
    main()
