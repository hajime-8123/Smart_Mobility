import rclpy
from rclpy.action import ActionClient
from fleet_management.action import FleetManagement
import click

@click.command()
@click.option("--fleet-size", type=int, help="Specify the fleet size.")
def main(fleet_size):
    rclpy.init()
    node = rclpy.create_node('fleet_management_cli')

    # Internally call the Action Client CLI
    action_client_script = 'fleet_management_client_cli.py'

    cmd = f'ros2 run my_package {action_client_script} --fleet-size {fleet_size}'
    result = os.system(cmd)

    if result != 0:
        node.get_logger().error('Failed to allocate and route vehicles')
    else:
        node.get_logger().info('Vehicles allocated and routed successfully')

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

