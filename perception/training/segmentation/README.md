# Training instructions

## Data labeling

1. Go to training repo https://app.roboflow.com/ben-o3uie/true-battlebots/annotate
1. "Upload More Images"
1. Select Folder and click upload
1. Assign them to yourself
1. Click "Start Annotating"
1. Label images. I recommend labeling the field once, copying to all images, then go back and label the robots.

## Create dataset

1. Click "Versions"
1. Click "Create New Version"
1. Skip all augmentation options
1. Click "Export Dataset"
1. Run the download command and unzip to `/media/storage/true-battlebot-media/training/labeled/`
1. Merge the dataset. Edit and run this file as appropriate. `python merge_robotflow_output.py`
1. Augment the dataset. Edit and run this file as appropriate. `python augment.py`
1. Split the dataset. Edit and run this file as appropriate. `python split_dataset.py`

## Run training

1. Run the training. Edit and run this file as appropriate. `train.py`
1. Run the training script. `./train.sh`
1. Check http://localhost:6006 for tensorboard
