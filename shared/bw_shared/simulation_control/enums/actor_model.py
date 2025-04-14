from enum import Enum


class ActorModel(str, Enum):
    MR_STABS_MK2 = "MR STABS MK2"
    MRS_BUFF_MK2 = "MRS BUFF MK2"
    MRS_BUFF_MK1 = "MRS BUFF B-03"
    MRS_BUFF_MK1_BIZARRO = "MRS BUFF B-03 Bizarro"
    MR_STABS_A_02 = "MR STABS A-02"
    REFEREE = "Referee"
    TRAINING_CAMERA = "Training Camera"
    TRACKING_CAMERA = "OAK-1"
    SLOW_CAMERA = "ZED 2i"
