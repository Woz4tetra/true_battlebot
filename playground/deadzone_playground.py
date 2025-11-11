import numpy as np

LOWER_DEADZONE_PERCENT = -7.0
UPPER_DEADZONE_PERCENT = 3.0


def deadzone_func(signed_percent):
    if signed_percent > 0:
        scaled_percent = signed_percent - UPPER_DEADZONE_PERCENT
    else:
        scaled_percent = signed_percent + LOWER_DEADZONE_PERCENT
    scaled_percent = min(100, max(-100, scaled_percent))

    return scaled_percent


if __name__ == "__main__":
    test_values = np.linspace(-20, 20, num=19)
    print("Input Percent | Scaled Percent")
    print("-------------------------------")
    for val in test_values:
        scaled = deadzone_func(val)
        print(f"    {val:6.1f}    |    {scaled:6.3f}")
