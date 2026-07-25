# complete code
import argparse
import sys
import rclpy
from rclpy.utilities import remove_ros_args

def main():
    parser = argparse.ArgumentParser(description='ros2_lingua_mock cli')
    parser.add_argument('--namespace', help='namespace for the service')
    args, _ = parser.parse_known_args()
    namespace = args.namespace

    # Remove ROS-specific arguments
    sys.argv = remove_ros_args(sys.argv)

    # Parse the remaining arguments
    parser.parse_args()

    # Create a ROS2 node
    node = rclpy.create_node('lingua_mock')

    # Check if the service exists in the namespace
    try:
        node.get_service(namespace + '/lingua/ground')
    except rclpy.exceptions.ServiceException as e:
        print(f"Service '{namespace + '/lingua/ground'}' not found")

if __name__ == '__main__':
    main()