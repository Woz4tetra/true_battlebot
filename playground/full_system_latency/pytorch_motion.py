#! /usr/bin/env python3

from pathlib import Path

import blobconverter
import torch
from torch import nn


class Model(nn.Module):
    def forward(self, img1, img2):
        image_width = img1.shape[2]
        image_height = img1.shape[1]
        area = image_width * image_height
        sum1 = torch.sum(img1, dim=0)
        sum2 = torch.sum(img2, dim=0)
        diff = torch.abs(torch.sub(sum1, sum2))
        total_diff = torch.sum(diff)
        norm_diff = (total_diff / area).item()
        norm_diff_tensor = torch.tensor([norm_diff], dtype=torch.float32)
        return norm_diff_tensor


class Model(nn.Module):
    def forward(self, img1, img2):
        sum1 = torch.sum(img1, dim=0)
        sum2 = torch.sum(img2, dim=0)
        return torch.sub(sum1, sum2)


# Define the expected input shape (dummy input)
shape = (3, 720, 720)
X = torch.ones(shape, dtype=torch.float32)

path = Path("out/")
path.mkdir(parents=True, exist_ok=True)
onnx_file = "out/diff.onnx"

print(f"Writing to {onnx_file}")
torch.onnx.export(
    Model(),
    (X, X),
    onnx_file,
    opset_version=12,
    do_constant_folding=True,
    input_names=["img1", "img2"],  # Optional
    output_names=["output"],  # Optional
)

# No need for onnx-simplifier here

# Use blobconverter to convert onnx->IR->blob
blobconverter.from_onnx(
    model=onnx_file, data_type="FP16", shaves=6, use_cache=False, output_dir="models", optimizer_params=[]
)
