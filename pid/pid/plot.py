import rclpy
from rclpy.node import Node
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.animation import PillowWriter
from std_msgs.msg import Float64, Float32MultiArray
import threading

data_array = [0, 0, 0, 0]

fig, ax = plt.subplots(1, 1)
fig.set_size_inches(5,5)


class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')

        self.subscription = self.create_subscription(
            Float32MultiArray,
            'usvpos',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        data_array = [msg.data[0], msg.data[1], msg.data[2], msg.data[3]]

        if msg.data[4] != 0.0 and msg.data[5] != 0.0:
            plt.plot([msg.data[2], msg.data[4]], [msg.data[3], msg.data[5]], '--g')

        if msg.data[0] != 0.0 and msg.data[1] != 0.0:
            plt.plot(msg.data[0], msg.data[1], '.b')
        plt.plot(msg.data[2], msg.data[3], '*r')
        
        plt.axis("equal")
        plt.draw()
        plt.pause(0.00000000001)
        # print(data_array)

def animate(i):
    print(i)
    ax.clear()
    # Get the point from the points list at index i
    # Plot that point using the x and y coordinates
    ax.plot(data_array[0], data_array[1], color='green', 
            label='original', marker='o')
    ax.plot(data_array[2], data_array[3], color='red', 
            label='original', marker='o')
    # Set the x and y axis to display a fixed range
    ax.set_xlim([-100, 100])
    ax.set_ylim([-100, 100])

def start_animation():
    print("Hi")
    plt.show()


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    # threading.Thread(target=start_animation).start()

    # ani = FuncAnimation(fig, animate,
                    # interval=500, repeat=False)
    plt.ion()
    plt.show()
    

    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()