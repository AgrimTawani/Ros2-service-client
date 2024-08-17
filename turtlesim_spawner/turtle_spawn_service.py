import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn
from geometry_msgs.msg import Twist

class TurtleSpawnService(Node):

    def __init__(self):
        super().__init__('turtle_spawn_service')
        self.srv = self.create_service(Spawn, 'spawn_turtle', self.spawn_turtle_callback)
        self.get_logger().info('Service server for spawning turtles has started.')

    def spawn_turtle_callback(self, request, response):
        self.get_logger().info(f'Received request to spawn turtle at ({request.x}, {request.y}) with name "{request.name}"')

        # Spawn the turtle
        client = self.create_client(Spawn, '/spawn')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response.name = future.result().name

        return response
    
    def set_velocity(self, name, vx, vy):
        # This function sets the velocity of the turtle
        topic_name = f'/{name}/cmd_vel'
        publisher = self.create_publisher(Twist, topic_name, 10)
        
        twist = Twist()
        twist.linear.x = vx
        twist.linear.y = vy
        publisher.publish(twist)
        
        self.get_logger().info(f'Set velocity of turtle "{name}" to vx: {vx}, vy: {vy}')

def main(args=None):
    rclpy.init(args=args)
    service = TurtleSpawnService()
    rclpy.spin(service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
