from bw_shared.pid.config import PidConfig


class PID:
    """
    Implements a PID controller with optional feedforward term and integral zone.
    """

    def __init__(self, config: PidConfig):
        """
        Config arguments:
        kp: Proportional gain.
        ki: Integral gain.
        kd: Derivative gain.
        kf: Feedforward gain.
        i_zone: Integral zone. If the error is within this zone, the integral term is
            accumulated. If not set, the integral term is always accumulated.
        i_max: Maximum value of the integral term. If not set, the integral term is not
            limited.
        tolerance: Error tolerance for which the controller is considered to have reached the
            setpoint. Output will be set to zero if the error is within this tolerance.
        """
        self.kp = config.kp
        self.ki = config.ki
        self.kd = config.kd
        self.kf = config.kf
        self.i_zone = config.i_zone
        self.i_max = config.i_max
        self.tolerance = config.tolerance
        self.i_accum = 0.0
        self.prev_error = 0.0

    def reset(self) -> None:
        """
        Reset the integral term and previous error.
        """
        self.i_accum = 0.0
        self.prev_error = 0.0

    def update(self, setpoint: float, measurement: float, dt: float) -> float:
        """
        Update the PID controller with the current setpoint and measurement.

        Args:
            setpoint: Desired value.
            measurement: Current value.
            dt: Time step.

        Returns:
            Output value.
        """
        error = setpoint - measurement
        if abs(error) < self.tolerance:
            return 0.0
        output = 0.0
        output += self._calculate_p(error)
        output += self._calculate_i(error)
        output += self._calculate_d(error, dt)
        output += self._calculate_f(setpoint)
        return output

    def _calculate_p(self, error: float) -> float:
        if self.kp == 0.0:
            return 0.0
        return self.kp * error

    def _calculate_i(self, error: float) -> float:
        if self.ki == 0.0:
            return 0.0
        if self.i_zone is None:
            self.i_accum += error
        elif abs(error) < self.i_zone:
            self.i_accum += error

        if self.i_max != 0.0:
            if self.i_accum > 0.0:
                self.i_accum = min(self.i_accum, self.i_max / self.ki)
            else:
                self.i_accum = max(self.i_accum, -self.i_max / self.ki)

        return self.ki * self.i_accum

    def _calculate_d(self, error: float, dt: float) -> float:
        if self.kd == 0.0:
            return 0.0
        if dt < 0.0:
            return 0.0
        output = self.kd * (error - self.prev_error) / dt
        self.prev_error = error
        return output

    def _calculate_f(self, setpoint: float) -> float:
        if self.kf == 0.0:
            return 0.0
        return self.kf * setpoint
