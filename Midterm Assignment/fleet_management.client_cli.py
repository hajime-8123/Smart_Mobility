import rclpy
from rclpy.action import ActionClient
from fleet_management.action import FleetManagement
import click

@click.command()
@click.option("--fleet-size", type=int, help="Specify the fleet size.")
def main(fleet_size):
    rclpy.init()
    node = rclpy.create_node('fleet_management_client')

    action_client = ActionClient(node, FleetManagement, 'fleet_management')

    while not action_client.wait_for_server(timeout_sec=1.0):
        node.get_logger().info('Action server not available, waiting...')

    goal = FleetManagement.Goal()
    goal.fleet_size = fleet_size

    future = action_client.send_goal_async(goal)

    rclpy.spin_until_future_complete(node, future)

    if future.result() is not None:
        action_result = future.result().result
        node.get_logger().info('Vehicle Routes: %s' % action_result.vehicle_routes)
    else:
        node.get_logger().error('Failed to get routes')

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

