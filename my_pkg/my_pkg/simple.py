import rclpy
# import the Node module from ROS2 Python library
from rclpy.node import Node
from colorama import Fore, Style

def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # print a message to the terminal
    print("\033[91mHello World!\033[0m")
    print(Fore.RED + "Hello World!" + Style.RESET_ALL)  # 빨간색 텍스트 
    print(Fore.MAGENTA + "Hello World!" + Style.RESET_ALL)  # 빨간색 텍스트 

    # shutdown the ROS communication
    rclpy.shutdown()

if __name__ == '__main__':
    main() # call the main function