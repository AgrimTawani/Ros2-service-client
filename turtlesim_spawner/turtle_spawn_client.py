import sys
import rclpy
import time
from rclpy.node import Node
from turtlesim.srv import Spawn
from geometry_msgs.msg import Twist

class TurtleSpawnClient(Node):

    def __init__(self):
        super().__init__('turtle_spawn_client')
        self.client = self.create_client(Spawn, 'spawn_turtle')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.get_logger().info('Connected to the service.')

    def send_request(self, x, y, name):
        request = Spawn.Request()
        request.x = x
        request.y = y
        request.name = name
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def set_velocity(self, name, vx, vy):
        publisher = self.create_publisher(Twist, f"/{name}/cmd_vel", 10)
        twist = Twist()
        twist.linear.x = vx
        twist.linear.y = vy
        
        publisher.publish(twist)
        self.get_logger().info(f'Set velocity of turtle "{name}" to vx: {vx}, vy: {vy}')

def main(args=None):
    rclpy.init(args=args)
    
    x = float(sys.argv[1])
    y = float(sys.argv[2])
    name = sys.argv[3]
    vx = float(sys.argv[4])
    vy = float(sys.argv[5])

    client = TurtleSpawnClient()
    response = client.send_request(x, y, name)
    client.set_velocity(name, vx, vy)
    
    print(f'Turtle "{response.name}" spawned at ({x}, {y}) with velocity ({vx}, {vy})')

    rclpy.shutdown()

if __name__ == '__main__':
    main()
