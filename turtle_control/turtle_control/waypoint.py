"""A demonstration Node for ME495."""
import math

from example_interfaces.msg import Int64
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from std_srvs.srv import Empty
from turtlesim_msgs.srv import SetPen, TeleportAbsolute
from turtle_interfaces.srv import Waypoints


class WayPoint(Node):
    """
    Node that demonstrates ROS 2.

    Publishes
    ---------
    count : example_interfaces/msg/Int64 - the count that is incremented

    Parameters
    ----------
    increment : Integer - the amount to increment the count by each time

    Services
    --------
    reset (std_srv/srv/Empty) - reset the count
    toggle (std_srv/srv/Toggle) - toggles between MOVING and STOPPED

    Subscribes
    ----------
    uncount (std_msgs/msg/Int64) - subtract a value from the count

    """

    def __init__(self):
        """Create WayPoint node."""
        super().__init__('waypoint')
        self.get_logger().info('WayPoint')
        self.declare_parameter('increment', 1)
        self._inc = self.get_parameter('increment').value
        self.personal_call_back_group = MutuallyExclusiveCallbackGroup()
        self._state = 0

        self._pub = self.create_publisher(Int64, 'count', 10)
        self._tmr = self.create_timer(0.011, self.timer_callback)
        self._tog = self.create_service(Empty, 'toggle', self.toggle_callback)

        self.reset = self.create_client(Empty, '/reset', callback_group=self.personal_call_back_group)
        self.set_pen = self.create_client(SetPen, 'turtle1/set_pen', callback_group=self.personal_call_back_group)
        self.teleport = self.create_client(TeleportAbsolute, 'turtle1/teleport_absolute', callback_group=self.personal_call_back_group)

        self.set_pen_msg = SetPen.Request(r=100,g=100, b=100, width=5)
        self.teleport_msg = TeleportAbsolute.Request()

        self._load = self.create_service(Waypoints, 'load', self.load_callback)
        self._sub = self.create_subscription(Int64, 'uncount', self.uncount_callback, 10)
        self._count = 0

    async def load_callback(self, request, response):
        self.get_logger().info('load_callback  called')

        response.distance = 0
        if request.waypoint is None:
            return response
        await self.reset.call_async(Empty.Request())

        self.set_pen_msg.off = 1
        await self.set_pen.call_async(self.set_pen_msg)

        previous_p = None
        next_p = None

        for index, position in enumerate(request.waypoint):
            if index > 0:
                next_p = (position.x, position.y)
            else:
                next_p = (position.x, position.y)
                previous_p = (position.x, position.y)

            #corner1_x, corner1_y, corner2_x, corner2_y = position
            corner1_x = position.x + 0.5
            corner1_y = position.y + 0.5
            corner2_x = position.x - 0.5
            corner2_y = position.y - 0.5

            self.teleport_msg.x, self.teleport_msg.y = corner1_x, corner1_y
            await self.teleport.call_async(self.teleport_msg)

            self.set_pen_msg.off = 0
            await self.set_pen.call_async(self.set_pen_msg)

            self.teleport_msg.x, self.teleport_msg.y = corner2_x, corner2_y
            await self.teleport.call_async(self.teleport_msg)
            self.set_pen_msg.off = 1
            await self.set_pen.call_async(self.set_pen_msg)

            ##second set of corners??
            corner1_x = position.x - 0.5
            corner1_y = position.y + 0.5
            corner2_x = position.x + 0.5
            corner2_y = position.y - 0.5

            self.teleport_msg.x, self.teleport_msg.y = corner1_x, corner1_y
            await self.teleport.call_async(self.teleport_msg)

            self.set_pen_msg.off = 0
            await self.set_pen.call_async(self.set_pen_msg)

            self.teleport_msg.x, self.teleport_msg.y = corner2_x, corner2_y
            await self.teleport.call_async(self.teleport_msg)
            self.set_pen_msg.off = 1
            await self.set_pen.call_async(self.set_pen_msg)

            response.distance  += math.dist(previous_p, next_p)
            previous_p = next_p


        self.get_logger().info(f'Distance traveled ? : {response.distance}')
        return response

    def toggle_callback(self, request, response):
        if self._state == 1:
            self.get_logger().info('Stopping')
            self._state = 0
        else:
            self.get_logger().info('Issuing Command!')
            self._state = 1
        return response

    def timer_callback(self):
        """Increment the count at a fixed frequency."""
        if self._state == 1:
            self.get_logger().debug('Timer Hit')
            self._pub.publish(Int64(data=self._count))
            self._count += self._inc

    def reset_callback(self, request, response):
        """Reset the count to zero."""
        self.get_logger().info('RESET!')
        self._count = 0
        return response

    def uncount_callback(self, uncount):
        """
        Subtract the value from count.

        Args:
        ----
        uncount : std_msgs/msg/Int64 - amount to reset the count

        """
        self.get_logger().debug(f'Uncount: {uncount.data}')
        self._count -= uncount.data


def main(args=None):
    """Entrypoint for the mynode ROS node."""
    rclpy.init(args=args)
    node = WayPoint()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    import sys
    main(sys.argv)