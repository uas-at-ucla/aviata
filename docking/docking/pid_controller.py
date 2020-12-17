class PIDController:
    def __init__(self, dt):
        self.dt = dt
        self.prev_errs = (0, 0, 0)
        self.sum_errs = (0, 0, 0)
        self.ki_dv_start = False
        self.ki_ev_start = False

    def get_velocities(self, x_err, y_err, alt_err):
        ku_nv = 2.15
        ku_ev = 2.15
        ku_dv = 3.5

        tu_nv = 1.3
        tu_ev = 0.90 # - 0.95
        tu_dv = 0.45

        kp_nv = 0.6 * ku_nv
        kp_ev = 0.6 * ku_ev # Ziegler-Nichols classic PID
        kp_dv = 0.6 * ku_dv # ZN

        kd_nv = 0.075 * ku_nv * tu_nv
        kd_ev = 0.15 # ZN
        kd_dv = 0.12 # ZN, minor manual tuning

        ki_nv = 1.2 * ku_nv / tu_nv
        ki_ev = 0.16 # manual tuning
        ki_dv = 3.60 # manual tuning

        east_velocity = x_err * kp_ev + (x_err - self.prev_errs[0]) / self.dt * kd_ev + self.sum_errs[0] * self.dt * ki_ev
        north_velocity = 0 # y_err * kp_ev + (y_err - prev_errs[1]) * kd_ev / dt + sum_errs[1] * dt * ki_nv
        down_velocity = alt_err * kp_dv + (alt_err - self.prev_errs[2]) / self.dt * kd_dv + self.sum_errs[2] * self.dt * ki_dv
        print(alt_err)

        # Set maximum speed
        east_velocity = self.abs_minmax(east_velocity, 0.5)
        north_velocity = self.abs_minmax(north_velocity, 0.5)
        down_velocity = self.abs_minmax(down_velocity, 0.5)

        if alt_err < 0.5:
            self.ki_dv_start = True
        if x_err < 0.5:
            self.ki_ev_start = True

        self.prev_errs = (x_err, y_err, alt_err)
        self.sum_errs = (
                            self.sum_errs[0] + x_err if self.ki_ev_start is True else 0, 
                            self.sum_errs[1] + y_err, 
                            self.sum_errs[2] + alt_err if self.ki_dv_start is True else 0
                        )

        return (east_velocity, north_velocity, down_velocity)

    def abs_minmax(self, val, minmax):
        if val > minmax: 
            val = minmax
        elif val < -minmax:
            val = -minmax
        return val