import rclpy
from rclpy.action import ActionServer
from fleet_management.msg import FleetManagement
from fleet_management.action import FleetManagementAction

class FleetManagementServer:
    def __init__(self):
        self.action_server = ActionServer(
            node,
            FleetManagementAction,
            'fleet_management',
            self.execute_callback)
    def execute_callback(self, goal_handle):
        feedback_msg = FleetManagement.Feedback()
        result_msg = FleetManagement.Result()

        fleet_size = goal_handle.request.fleet_size

        # Perform fleet management logic (e.g., allocation and routing)
        # Replace this with your actual logic
        vehicle_routes = ["Route1", "Route2"]
        completion_percentage = 100.0

        result_msg.vehicle_routes = vehicle_routes
        feedback_msg.completion_percentage = completion_percentage

        if fleet_size < 0:
            goal_handle.canceled()
        else:
            goal_handle.succeed(result_msg, "Fleet management completed.")

        # Optional: Publish feedback while processing
        goal_handle.publish_feedback(feedback_msg)
        
def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('fleet_management_server')
    
    fleet_management_server = FleetManagementServer(node)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    fleet_management_server.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

