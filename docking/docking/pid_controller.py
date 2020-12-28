class PIDController:
    def __init__(self, dt):
        self.dt = dt
        self.prev_errs = (0, 0, 0)
        self.sum_errs = (0, 0, 0)
        self.ki_dv_start = False
        self.ki_ev_start = False
        self.ki_nv_start = False

    def get_velocities(self, x_err, y_err, alt_err):
        ku_nv = 2.6
        ku_ev = 2.15
        ku_dv = 3.5

        # tu_nv = 0.75
        # tu_ev = 0.90
        # tu_dv = 0.45

        # Ziegler-Nichols classic PID
        kp_nv = 0.6 * ku_nv 
        kp_ev = 0.6 * ku_ev
        kp_dv = 0.6 * ku_dv

        # ZN: 0.075 * ku_nv * tu_nv, + minor manual tuning
        kd_nv = 0.15 
        kd_ev = 0.15
        kd_dv = 0.12

        # manual tuning
        ki_nv = 0 
        ki_ev = 0.16 
        ki_dv = 3.60

        east_velocity = x_err * kp_ev + (x_err - self.prev_errs[0]) / self.dt * kd_ev + self.sum_errs[0] * self.dt * ki_ev
        north_velocity = y_err * kp_nv + (y_err - self.prev_errs[1]) / self.dt * kd_nv + self.sum_errs[1] * self.dt * ki_nv
        down_velocity = alt_err * kp_dv + (alt_err - self.prev_errs[2]) / self.dt * kd_dv + self.sum_errs[2] * self.dt * ki_dv

        # Set maximum speed
        east_velocity = self.abs_minmax(east_velocity, 0.5)
        north_velocity = self.abs_minmax(north_velocity, 0.5)
        down_velocity = self.abs_minmax(down_velocity, 0.5)

        if alt_err < 0.5:
            self.ki_dv_start = True
        if x_err < 0.5:
            self.ki_ev_start = True
        if y_err < 0.5:
            self.ki_nv_start = True

        self.prev_errs = (x_err, y_err, alt_err)
        self.sum_errs = (
                            self.sum_errs[0] + x_err if self.ki_ev_start is True else 0, 
                            self.sum_errs[1] + y_err if self.ki_nv_start is True else 0, 
                            self.sum_errs[2] + alt_err if self.ki_dv_start is True else 0
                        )

        return (east_velocity, north_velocity, down_velocity)

    def abs_minmax(self, val, minmax):
        if val > minmax: 
            val = minmax
        elif val < -minmax:
            val = -minmax
        return val