import rclpy
from rclpy.node import Node
from carla_msgs.msg import CarlaEgoVehicleControl

class VehicleControlPublisher(Node):

    def __init__(self):
        super().__init__('vehicle_control_publisher')
        self.publisher = self.create_publisher(CarlaEgoVehicleControl, '/carla/hero/vehicle_control_cmd', 10)
        self.timer = self.create_timer(0.1, self.publish_control)  # Publish every 0.1 seconds
        self.control = CarlaEgoVehicleControl()

    def publish_control(self):
        # Set throttle to 1.0 for full throttle and keep steering straight
        self.control.throttle = 1.0
        self.control.steer = 0.0
        self.control.brake = 0.0
        self.control.hand_brake = False
        self.control.reverse = False
        self.control.gear = 1
        self.control.manual_gear_shift = False
        
        self.publisher.publish(self.control)
        self.get_logger().info(f'Publishing: {self.control}')

def main(args=None):
    rclpy.init(args=args)
    control_publisher = VehicleControlPublisher()
    rclpy.spin(control_publisher)
    control_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
