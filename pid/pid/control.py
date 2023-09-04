# Add velocity vector to further enhance dealing with inertia 
# Thruster rotation is done and commented, remove comment for rotation
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from std_msgs.msg import Float64, Float32MultiArray
from pid.autotune import PIDAutotune
from tf2_msgs.msg import TFMessage
from pid.QuatToEuler import *

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')

        # All the points below specify the path
        self.path = [[40, -1380], [100, -1320], [50, -1400], [150, -1320]]
        self.path = [[120, -1250], [120, -1400], [20, -1400], [20, -1300]]
        self.path = [[-120, -1250], [120, -1400], [20, -1400], [20, -1300]]

        self.prev_x = 0
        self.prev_y = 0

        # Angular speed control gains # Not used, a bang bang controller is used instead
        self.kp_as = 178.1
        self.ki_as = 30.6
        self.kd_as = 563.4
        self.i_as = 0.0
        self.prev_err_as = 0.0

        # Linear speed control gains
        self.kp_ls = 10
        self.ki_ls = 0
        self.kd_ls = 0
        self.i_ls = 0.0
        self.prev_err_ls = 0.0

        # Alignment control gains
        self.kp_a = 2
        self.ki_a = 0.0001
        self.ki_a = 0
        self.i_a = 0
        self.kd_a = 0
        self.prev_err_a = 0.0

        # Angular position control gains
        self.kp_p = 0.02
        self.ki_p = 0.0001
        self.kd_p = 0.0
        self.i_p = 0.0
        self.prev_err_p = 0.0

        # Velocity Vector Variables
        self.prev_pos_x = 0
        self.prev_pos_y = 0
        self.velocity_angle = 0
        self.beta_ang = 0

        # Target position

        self.target_x, self.target_y = self.path[-1]


        self.first = True # To only save the starting points

        self.state = 1 # The states of the bang bang controller

        self.timestamp = 0 # For the autotuner

        self.translation = [0, 0, 0] # USV translation
        self.starting_points = [0, 0]

        self.counter = 0

        # ROS Related Variables

        # Publishers
        self.publisher_ = self.create_publisher(Float64, 'usv/left/thrust/cmd_thrust', 10)
        self.publisher2_ = self.create_publisher(Float64, 'usv/right/thrust/cmd_thrust', 10)
        self.publisher3_ = self.create_publisher(Float32MultiArray, 'usvpos', 10) # To publish data for plotting
        self.publisher4_ = self.create_publisher(Float64, '/usv/right/thrust/joint/cmd_pos', 10)
        self.publisher5_ = self.create_publisher(Float64, '/usv/left/thrust/joint/cmd_pos', 10)
        # Subscribers
        self.subscription = self.create_subscription(
            Imu,
            '/usv/imu/data',
            self.listener_callback,
            10)   
        self.subscription2 = self.create_subscription(
            TFMessage,
            '/usv/pose_static',
            self.listener_callback2,
            10)
        
        self.timer_period = 0.01  # seconds, main loop period
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        self.euler = [0, 0, 0]
        self.angular_velocity = 0 # Yaw angular speed

        self.autotuner = PIDAutotune(
            2, 500, 1, out_min=-2000,
            out_max=2000, time=lambda: self.timestamp)
        
        self.path_counter = 0
            
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg): # Reads from imu topic
        self.angular_velocity = msg.angular_velocity.z
        self.euler = toEulerAngle(Quaterniond(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w))

    def listener_callback2(self, msg): # Reads from USV position topic
        for i in msg.transforms:
            if i.header.frame_id == "coast":
                self.translation = [i.transform.translation.x, i.transform.translation.y, i.transform.translation.z]
                if self.first:
                    self.starting_points = [self.translation[0], self.translation[1]]
                    self.first = False
                break

    def timer_callback(self): # Main loop

        matplotlib_msg = Float32MultiArray()
        msg = Float64()

        # Rotating the angle of the orientation 90 degrees for alignment
        yaw = self.euler[2] - 90 
        if yaw < 0:
            yaw = 360 + yaw

        starting_y = self.starting_points[0]
        starting_x = -self.starting_points[1]

        current_y = self.translation[0]
        current_x = -self.translation[1]

        translated_x = current_x - starting_x
        translated_y = current_y - starting_y

        if current_x != self.prev_x:
            self.counter = self.counter + 1
        
        self.prev_x = starting_x
        self.prev_y = starting_y

        # Calculating the angle between the starting point and the target point
        # This creates the path that needs to be followed
        angle = math.atan2(self.target_y - starting_y, self.target_x - starting_x)
        angle = angle * 180 / math.pi

        if angle < 0: # Changing the range from -180,180 to 0,360
            angle += 360 

        # Rotating the reference frame such that the line from the starting point 
        # to the target point aligns with the x-axis
        transformed_x = translated_x * math.cos(-angle * math.pi / 180) - translated_y * math.sin(-angle * math.pi / 180)
        transformed_y = translated_x * math.sin(-angle * math.pi / 180) + translated_y * math.cos(-angle * math.pi / 180)

        transformed_goal_x = (self.target_x - starting_x) * math.cos(-angle * math.pi / 180) - (self.target_y - starting_y) * math.sin(-angle * math.pi / 180)
        transformed_goal_y = (self.target_x - starting_x) * math.sin(-angle * math.pi / 180) + (self.target_y - starting_y) * math.cos(-angle * math.pi / 180)

        transformed_distance_to_x = transformed_goal_x - transformed_x
        
        # USV alignment control (Path following)

        dt = 1
        err = -transformed_y # Using the cross track error

        p = self.kp_a * err
        self.i_a += err * self.ki_a * dt

        # if abs(self.i_p) > 3: # Anti wind up
        #     self.i_p = 3.0 # Change to accomadate with the sign

        d = self.kd_a * (err - self.prev_err_a) / dt

        self.prev_err_a = err

        pid_out = p + self.i_a + d
        # if pid_out > 3:
        #     pid_out = 3.0

        # Limiting the integral part (anit wind-up)
        if self.kd_a > 90:
            self.kd_a = 90
        elif self.kd_a < -90:
            self.kd_a = -90

        # Limit the output to 90 degrees
        if pid_out > 90:
            pid_out = 90
        elif pid_out < -90:
            pid_out = -90

        # Rotating thrusters # Remove comment for thruster rotation
        # new_yaw = yaw
        # rotating_margin = 45
        # if angle - rotating_margin < 0 and yaw > 360 - rotating_margin:
        #     new_yaw = new_yaw - 360
        # if abs(transformed_y) > 5 and new_yaw < angle + rotating_margin and new_yaw > angle - rotating_margin:
        #     if transformed_y > 0:
        #         msg.data = -0.05
        #         self.publisher4_.publish(msg)
        #         self.publisher5_.publish(msg)
        #         print("yes")
        #     elif transformed_y < 0:
        #         print("yes")
        #         msg.data = 0.05
        #         self.publisher4_.publish(msg)
        #         self.publisher5_.publish(msg)
        # else:
        #     msg.data = 0.0
        #     self.publisher4_.publish(msg)
        #     self.publisher5_.publish(msg)

        # beta = current angle - velocity vector angle 

        # Calculating velocity vector

        if self.prev_pos_x - current_x != 0:
            self.velocity_angle = math.atan2((-current_y + self.prev_pos_y),  (current_x - self.prev_pos_x))
            self.velocity_angle = self.velocity_angle * 180 / math.pi
            self.velocity_angle = -self.velocity_angle

            self.prev_pos_y = current_y
            self.prev_pos_x = current_x

            # self.velocity_angle = -self.velocity_angle

            if self.velocity_angle < 0: # Changing the range from -180,180 to 0,360
                self.velocity_angle += 360         

            self.beta_ang = self.velocity_angle - yaw
            if self.beta_ang > 180:
                self.beta_ang = 360 - self.beta_ang
            elif self.beta_ang < -180:
                self.beta_ang = -360 - self.beta_ang

            if self.beta_ang > 45:
                self.beta_ang = 45

            if self.beta_ang < -45:
                self.beta_ang = -45

        desired_angle = angle + pid_out - self.beta_ang # need to add angle beta

        # Publishing data for plotting
        data = [float(current_x), float(current_y), float(self.target_x), float(self.target_y), float(starting_x), float(starting_y)]
        matplotlib_msg.data = data
        self.publisher3_.publish(matplotlib_msg)

        print("\n\t· Yaw: " + str(yaw))
        # print("\n\t· distance to goal x: " + str(transformed_distance_to_x))
        print("\t· Desired angle: " + str(desired_angle))
        print("\t· angle: " + str(angle))
        print("\t· velocity angle: " + str(self.velocity_angle))
        print("\t· Beta: " + str(self.beta_ang))
        # print("\n\t· trans x: " + str(transformed_x))
        print("\t· trans y: " + str(transformed_y))
        print("\t· counter: " + str(self.counter))
        # print("\n\t· x: " + str(current_x))
        # print("\t· y: " + str(current_y))
        # print("\n\t· trans goal x: " + str(transformed_goal_x))
        # print("\t· trans goal y: " + str(transformed_goal_y))
        # print("\n· Alignment intergrator: " + str(self.i_a))


        # Angle control
        setpoint = desired_angle # angle setpoint
        # setpoint = 0


        if yaw < setpoint:
            angle = 360 - setpoint + yaw
        else:
            angle = yaw - setpoint

        if angle > 180:
            err = 360 - angle

            p = self.kp_p * err

            self.i_p += err * self.ki_p * self.timer_period
            if self.i_p > 3:
                self.i_p = 3.0

            d = self.kd_p * (err - self.prev_err_p) / self.timer_period

            self.prev_err_p = err

            msg.data = p + self.i_p + d

            if msg.data > 3:
                msg.data = 3.0

            # self.publisher2_.publish(msg)
            # msg.data = -msg.data
            # self.publisher_.publish(msg)

        else:
            err = angle

            p = self.kp_p * err
            self.i_p += err * self.ki_p * self.timer_period
            if self.i_p > 3:
                self.i_p = 3.0
            d = self.kd_p * (err - self.prev_err_p) / self.timer_period

            self.prev_err_p = err

            msg.data = p + self.i_p + d
            
            # self.publisher_.publish(msg)
            msg.data = -msg.data # to limit the output angular velocity
            if msg.data > 3:
                msg.data = 3.0           
            # self.publisher2_.publish(msg)

        
        setpoint = msg.data # Angle control output

        out_limit = 500.0 # Bang bang controller on and off limit -500 and 500

        # Angular speed control
        noiseband = 0.1


        if (self.state == 1
                and self.angular_velocity > setpoint + noiseband):
            self.state = 0

        if (self.state == 0
                and self.angular_velocity < setpoint - noiseband):
            self.state = 1

        if self.state == 1:
            msg.data = out_limit

        if self.state == 0:
            msg.data = -out_limit

        angular_speed_out = msg.data

        # Linear Speed control

        dt = 1
        err = transformed_distance_to_x

        p = self.kp_ls * err
        self.i_ls += err * self.ki_ls * dt

        # if abs(self.i_p) > 3: # Anti wind up
        #     self.i_p = 3.0 # Change to accomadate with the sign

        d = self.kd_ls * (err - self.prev_err_ls) / dt

        self.prev_err_ls = err

        pid_out = p + self.i_ls + d
        if pid_out > 500:
            pid_out = 500.0

        linear_speed_command = pid_out

        linear_speed_command -= abs(transformed_y) * 10 # Lowering the speed when going farther from the center line

        msg.data += linear_speed_command # 500 is used for speed control, change later for forward control

        if (msg.data < 50):
            msg.data = 50.0

        self.publisher2_.publish(msg) # Remove later
        msg.data = -angular_speed_out + linear_speed_command
        self.publisher_.publish(msg)

        if (abs(transformed_distance_to_x) < 2):
            self.target_x = self.path[self.path_counter][0]
            self.target_y = self.path[self.path_counter][1]
            # self.starting_points[1] = -current_x
            # self.starting_points[0] = current_y

            prev_path_counter = self.path_counter - 1
            if self.path_counter == -1:
                prev_path_counter = len(self.path) - 1
            self.starting_points[1] = -self.path[prev_path_counter][0]
            self.starting_points[0] = self.path[prev_path_counter][1]

            self.path_counter = (self.path_counter + 1) % len(self.path)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

