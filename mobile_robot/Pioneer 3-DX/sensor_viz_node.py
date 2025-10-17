import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import matplotlib.pyplot as plt

class SensorVisualizer(Node):
    def __init__(self):
        super().__init__('sensor_viz_node')

        # Initialize sensor readings
        self.sensor_readings = [1.0] * 16
        self.sensor_indices = list(range(1, 17))

        # Subscribe to 16 Float32 topics published by Lua
        for i in range(1, 17):
            topic = f'/ultrasonic_sensor_{i:02d}_float'
            self.create_subscription(Float32, topic, lambda msg, idx=i-1: self.callback(msg, idx), 10)

        # Set up live matplotlib plot
        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.bar_plot = self.ax.bar(self.sensor_indices, self.sensor_readings)
        self.ax.set_ylim(0, 1.0)
        self.ax.set_xlabel('Sensor Number')
        self.ax.set_ylabel('Distance (m)')
        self.ax.set_title('Ultrasonic Sensor Readings')

    def callback(self, msg, idx):
        # Update the distance for the corresponding sensor
        self.sensor_readings[idx] = msg.data

        # Update the plot
        self.ax.clear()
        self.ax.bar(self.sensor_indices, self.sensor_readings)
        self.ax.set_ylim(0, 1.0)
        self.ax.set_xlabel('Sensor Number')
        self.ax.set_ylabel('Distance (m)')
        self.ax.set_title('Ultrasonic Sensor Readings')
        plt.pause(0.01)

def main(args=None):
    rclpy.init(args=args)
    node = SensorVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
