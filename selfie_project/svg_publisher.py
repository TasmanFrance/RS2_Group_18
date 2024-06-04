import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SVGPublisher(Node):
    def __init__(self):
        super().__init__('svg_publisher')
        self.publisher_ = self.create_publisher(String, 'svg_path_data', 10)
        self.timer = self.create_timer(1.0, self.publish_svg_data)

    def publish_svg_data(self):
        from svg_reader import read_svg_paths
        import os

        svg_file_path = os.path.join(os.getcwd(), "test.svg")
        paths = read_svg_paths(svg_file_path)

        for path_data in paths:
            msg = String()
            msg.data = path_data
            self.publisher_.publish(msg)
            self.get_logger().info(f'Published SVG path data: {path_data}')

def main(args=None):
    rclpy.init(args=args)
    svg_publisher = SVGPublisher()
    rclpy.spin(svg_publisher)
    svg_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()