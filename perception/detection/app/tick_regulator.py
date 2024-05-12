import time
from typing import Generator


def time_since_start() -> float:
    return time.perf_counter()


def regulate_tick(sample_rate: float) -> Generator[float, None, None]:
    """
    Regulates the tick rate of the loop to the given sample rate.

    Example usage:
    for dt in regulate_tick(60.0):
        # dt is the time since the last tick in seconds
        do_stuff()  # called at 60Hz indefinitely
    """
    tick_delay = 1.0 / sample_rate
    dt = 0.0
    tick_start = time_since_start()
    prev_time = tick_start

    # sleep for one tick so delta time is consistent
    time.sleep(tick_delay)

    while True:
        tick_start = time_since_start()
        dt = tick_start - prev_time

        yield dt

        tick_stop = time_since_start()
        prev_time = tick_start
        tick_duration = tick_stop - tick_start
        remaining_time = tick_delay - tick_duration
        if remaining_time > 0.0:
            time.sleep(remaining_time)
