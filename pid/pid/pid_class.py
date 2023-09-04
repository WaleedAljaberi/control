
class Quaterniond():
    def __init__(self,in_x,in_y,in_z,in_w):
        self.x = in_x
        self.y = in_y
        self.z = in_z
        self.w = in_w

class PID():
    def __init__(self, kp = 0, ki = 0, kd = 0, i_limit = [0, 0], pid_limit = [0, 0]):

        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.i = 0
        self.prev_err = 0

        self.i_limit = i_limit
        self.pid_limit = pid_limit

    def compute(self, err, dt):

        p = self.kp * err
        self.i += err * self.ki * dt
        d = self.kd * (err - self.prev_err) / dt

        self.prev_err = err

        out = p + self.i + d

        if self.i > 90 and self.i_limit[0] != 0:
            self.i = 90
        elif self.i < -90 and self.i_limit[1] != 0:
            self.i = -90

        if self.i > 90 and self.i_limit[0] != 0:
            self.i = 90
        elif self.i < -90 and self.i_limit[1] != 0:
            self.i = -90

        return out



    
if __name__ == '__main__':
    print("Yes")

    pid = PID()
    print(pid.kp)
