import math

class Quaterniond():
    def __init__(self,in_x,in_y,in_z,in_w):
        self.x = in_x
        self.y = in_y
        self.z = in_z
        self.w = in_w

    def __str__(self):
        return "[" + str(round(self.x,3)) + ", " + str(round(self.y,3)) + ", " + str(round(self.z,3)) + ", " + str(round(self.w,3)) +"]" 


def toQuaternion(pitch, roll, current_angular_vel, log = True):
    # Abbreviations for the various angular functions
    cy = math.cos(current_angular_vel * 0.5)
    sy = math.sin(current_angular_vel * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)

    q = Quaterniond(0,0,0,0)
    q.w = cy * cr * cp + sy * sr * sp
    q.x = cy * sr * cp - sy * cr * sp
    q.y = cy * cr * sp + sy * sr * cp
    q.z = sy * cr * cp - cy * sr * sp

    if log:
        print("Transform Euler angles to quaternion(x,y,z,w):")
        print("- Input:")
        print("\t· Roll: " + str(roll))
        print("\t· Pitch: " + str(pitch))
        print("\t· current_angular_vel: " + str(current_angular_vel))
        print("Output:\n\t· Quat: " + str(q))

    return q

# Input [x,y,z,w] list with quat
def toEulerAngle(q, log = True):
    # roll (x-axis rotation)
    sinr_cosp = +2.0 * (q.w * q.x + q.y * q.z)
    cosr_cosp = +1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # pitch (y-axis rotation)
    sinp = +2.0 * (q.w * q.y - q.z * q.x)
    if (math.fabs(sinp) >= 1):
        pitch = math.copysign(math.pi / 2, sinp) # use 90 degrees if out of range
    else:
        pitch = math.asin(sinp)

    # current_angular_vel (z-axis rotation)
    siny_cosp = +2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = +1.0 - 2.0 * (q.y * q.y + q.z * q.z)  
    current_angular_vel = math.atan2(siny_cosp, cosy_cosp)

    current_angular_vel = current_angular_vel + math.pi
    current_angular_vel = current_angular_vel * 360 / (2 * math.pi) 
    log = False
    if log:
        print("Transform quaternion(x,y,z,w) to Euler angles:")
        print("- Input:\n\t· Quat: " + str(q))
        print("Output:")
        print("\t· Roll: " + str(roll))
        print("\t· Pitch: " + str(pitch))
        print("\t· Yaw: " + str(current_angular_vel))
    return [roll,pitch,current_angular_vel]