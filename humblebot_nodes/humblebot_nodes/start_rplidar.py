import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty

class StartRplidarServiceClient(Node):
    def __init__(self):
        super().__init__('start_motor_service_client')
        self.client = self.create_client(Empty, '/start_motor')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for the /start_motor service...')
        self.send_request()

    def send_request(self):
        req = Empty.Request()
        self.future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, self.future)

def main(args=None):
    rclpy.init(args=args)
    start_motor_service_client = StartRplidarServiceClient()
    start_motor_service_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
