import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
import math

class Quaterniond():
    def __init__(self, in_x, in_y, in_z, in_w):
        self.x = in_x
        self.y = in_y
        self.z = in_z
        self.w = in_w

    def __str__(self):
        return "[" + str(round(self.x, 3)) + ", " + str(round(self.y, 3)) + ", " + str(round(self.z, 3)) + ", " + str(round(self.w, 3)) + "]"

def toQuaternion(pitch, roll, current_angular_vel, log=True):
    # Abbreviations for the various angular functions
    cy = math.cos(current_angular_vel * 0.5)
    sy = math.sin(current_angular_vel * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)

    q = Quaterniond(0, 0, 0, 0)
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
def toEulerAngle(q, log=True):
    # roll (x-axis rotation)
    sinr_cosp = +2.0 * (q.w * q.x + q.y * q.z)
    cosr_cosp = +1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # pitch (y-axis rotation)
    sinp = +2.0 * (q.w * q.y - q.z * q.x)
    if (math.fabs(sinp) >= 1):
        pitch = math.copysign(math.pi / 2, sinp)  # use 90 degrees if out of range
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
    return [roll, pitch, current_angular_vel]

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')

        self.kp_yaw = 0.0  # Yaw control proportional gain
        self.ki_yaw = 0.0  # Yaw control integral gain
        self.kd_yaw = 0.0  # Yaw control derivative gain
        self.i_yaw = 0.0  # Yaw control integral term
        self.prev_err_yaw = 0.0  # Yaw control previous error

        self.target_yaw = 90.0  # Desired yaw angle in degrees

        # Add the tolerance_yaw attribute
        self.tolerance_yaw = 1.0  # Adjust the value as needed


        self.min_thrust = 100  # Minimum thruster value
        self.max_thrust = 1500  # Maximum thruster value

        self.publisher_ = self.create_publisher(Float64, 'usv/left/thrust/cmd_thrust', 10)
        self.publisher2_ = self.create_publisher(Float64, 'usv/right/thrust/cmd_thrust', 10)
        self.subscription = self.create_subscription(
            Imu,
            '/usv/imu/data',
            self.listener_callback,
            10)
        self.euler = [0, 0, 0]

        self.timer_period = 0.01  # Timer period in seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.autotune_stage = 0  # Autotune stage (0: Proportional, 1: Integral, 2: Derivative)
        self.autotune_step = 0  # Autotune step
        self.autotune_direction = 1  # Autotune direction (-1: Reverse, 1: Forward)
        self.autotune_output = []  # List to store autotune output

    def listener_callback(self, msg):
        self.euler = toEulerAngle(Quaterniond(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w))

    def timer_callback(self):
        current_yaw = self.euler[2]
        print("Current Yaw:", current_yaw)
        print("Desired Yaw:", self.target_yaw)

        # Calculate the error between the current yaw and the target yaw
        error_yaw = self.target_yaw - current_yaw

        # Adjust the error_yaw to be within the range [-180, 180)
        if error_yaw >= 180:
            error_yaw -= 360
        elif error_yaw < -180:
            error_yaw += 360

        # Check if the error is within the tolerance range
        if abs(error_yaw) <= self.tolerance_yaw:
            # Stop the boat since it has reached the desired yaw
            self.stop_boat()
        else:
            # PID control for yaw angle
            p_yaw = self.kp_yaw * error_yaw
            self.i_yaw += error_yaw * self.ki_yaw * self.timer_period
            d_yaw = self.kd_yaw * (error_yaw - self.prev_err_yaw) / self.timer_period

            self.prev_err_yaw = error_yaw

            # Calculate the thrust values for left and right thrusters
            thrust_left = p_yaw + self.i_yaw + d_yaw

            # Determine the direction of rotation
            if error_yaw > 0:
                # Overshot the desired yaw, rotate in the opposite direction
                thrust_right = -self.max_thrust
            else:
                thrust_right = self.max_thrust

            # Limit the thrust values within the range of 100 to 1500
            thrust_left = max(min(thrust_left, self.max_thrust), self.min_thrust)
            thrust_right = max(min(thrust_right, self.max_thrust), self.min_thrust)

            # Convert thrust values to float
            thrust_left = float(thrust_left)
            thrust_right = float(thrust_right)

            # Publish the thrust values to the left and right thrusters
            left_thruster_msg = Float64()
            right_thruster_msg = Float64()

            left_thruster_msg.data = thrust_left
            right_thruster_msg.data = thrust_right

            self.publisher_.publish(left_thruster_msg)
            self.publisher2_.publish(right_thruster_msg)

    

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
