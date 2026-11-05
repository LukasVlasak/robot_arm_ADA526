import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse

from robot_interfaces.action import Precison.action


class PrecisionNode(Node):

    def __init__(self):
        super().__init__('precision_node')

        self._action_server = ActionServer(
            self,
            Precison,
            'precision',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

        self.get_logger().info('Action server started')

    def goal_callback(self, goal_request):
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Cancel request received')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Executing movement')
        
        x = goal_handle.request.x
        y = goal_handle.request.y
        z = goal_handle.request.z
        
        try:
        
        except:
            goal_handle.abort()

            result = MoveToPosition.Result()
            result.success = False
            result.message = 'IK failed'

            return result
        
        
        # 3. Send motor angles to Motor Node
        # TODO: Send motor action goal

        # 4. Wait for motor execution
        # TODO: Wait for motor result

        # 5. Return final result
        result = MoveToPosition.Result()
        result.success = True
        result.message = 'Position reached'

        goal_handle.succeed()

        return result



def main(args=None):

    rclpy.init(args=args)

    node = MyNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

