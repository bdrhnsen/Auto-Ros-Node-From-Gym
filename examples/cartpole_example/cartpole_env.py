#!/usr/bin/env python3
import rclpy
from ros_gym_automation.cli.main import generate_env


def main():
    """Generate and run CartPole environment node."""
    # Generate environment node
    generate_env('CartPole-v1', 'cartpole_example', package_name='cartpole_demo')
    
    # Initialize ROS
    rclpy.init()
    try:
        # Create and run the node
        from cartpole_demo.nodes.cartpole_v1_node import CartPole_v1Node
        node = CartPole_v1Node()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main() 