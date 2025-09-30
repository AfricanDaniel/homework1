"""Waypoint Node for HW 1"""
import math

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node

from geometry_msgs.msg import Twist, Vector3
from std_srvs.srv import Empty
from turtle_interfaces.msg import ErrorMetric
from turtle_interfaces.srv import Waypoints
from turtlesim_msgs.msg import Pose
from turtlesim_msgs.srv import SetPen, TeleportAbsolute


class WayPoint(Node):
    """
    Waypoint Node interacts with the turtle sim

    Publishes
    ---------
    Twist : The linear and angular velocity of the turtle
    ErrorMetric : the metrics returned at the end of each loop

    Parameters
    ----------
    tolerance : the tolerance that the error has to be within
    frequency : how frequent the timer is publishing

    Services
    --------
    load (std_srv/srv/Waypoints) - Loads the waypoints passed in
    toggle (std_srv/srv/Toggle) - toggles between MOVING and STOPPED

    Subscribes
    ----------
    pose (turtlesim_msgs/msg/Pose) - The position of the turtle

    """

    def __init__(self):
        """Create WayPoint node."""
        super().__init__('waypoint')
        self.get_logger().info('WayPoint')

        """initializing all parameters"""
        # state determines if toggle is on or off, 0=off, 1=on
        self._state = 0
        self.velocity = 10.0
        self.turtle_pose = Pose()
        self._waypoints = None

        self.tolerance = self.declare_parameter('tolerance', 0.05)
        self.tolerance = float(self.get_parameter('tolerance').value)
        self.declare_parameter('frequency', 100)
        frequency = self.get_parameter('frequency').value

        """initializing all helper parameters"""
        self.complete_loops = 0
        self.actual_distance = 0.0
        self.prev_pose = None
        self.planned_distance = 0.0
        #personal call back group to avoid blocking
        self.personal_call_back_group = MutuallyExclusiveCallbackGroup()

        """Placing all the publishers in one location"""
        self.pub_twist = self.create_publisher(Twist, "turtle1/cmd_vel", 10)
        self.pub_metrics = self.create_publisher(ErrorMetric, "/loop_metrics", 10)

        """Placing all the subscribers in one location"""
        self.turtle_live_listener = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)

        """Placing all the Services in one location"""
        self._load = self.create_service(Waypoints, 'load', self.load_callback)
        self._tog = self.create_service(Empty, 'toggle', self.toggle_callback)

        """Placing all the Clients in one location"""
        self.reset = self.create_client(Empty, '/reset', callback_group=self.personal_call_back_group)
        self.set_pen = self.create_client(SetPen, 'turtle1/set_pen', callback_group=self.personal_call_back_group)
        self.teleport = self.create_client(TeleportAbsolute, 'turtle1/teleport_absolute',
                                           callback_group=self.personal_call_back_group)

        """All remainders"""
        self._tmr = self.create_timer(1/frequency, self.timer_callback)
        self.set_pen_msg = SetPen.Request(r=100,g=100, b=100, width=5)
        self.teleport_msg = TeleportAbsolute.Request()


    def timer_callback(self):
        """Increment the count at a fixed frequency."""
        if self._state == 1:

            if self.turtle_pose is None:
                self.get_logger().info("POSE no recieved yet")
                return

            """Get the next waypoint (x, y) we are trying to go to"""
            dest =  self._waypoints[self._waypoints_index]
            destination_x = dest.x
            destination_y = dest.y

            """Get the difference between destination and current pos"""
            dy = destination_y - self.turtle_pose.y
            dx = destination_x - self.turtle_pose.x

            distance_to_destination = math.sqrt(dx * dx + dy * dy)
            difference_in_theta = math.atan2(dy, dx) - self.turtle_pose.theta

            ####
            while difference_in_theta > math.pi:
                difference_in_theta = difference_in_theta - 2.0 * math.pi
            while difference_in_theta < -math.pi:
                difference_in_theta = difference_in_theta + 2 * math.pi
            ####


            if distance_to_destination < self.tolerance:

                self.get_logger().info("Destination reached")
                self._waypoints_index += 1

                if self._waypoints_index >= len(self._waypoints):
                    #self._state = 0

                    self.complete_loops = self.complete_loops + 1
                    self.error = self.actual_distance - self.planned_distance * self.complete_loops

                    self.pub_metrics.publish(ErrorMetric(complete_loops=int(self.complete_loops), actual_distance=float(self.actual_distance),
                                                         error=float(self.error)))

                    self.get_logger().info(f"[loop {self.complete_loops} complete] planned_distance={self.planned_distance*self.complete_loops}"
                                           f" actual_distance={self.actual_distance} error={self.error}")

                    self._waypoints_index = 0

            if abs(difference_in_theta) > 0.1:
                self.linear_velocity = 0.0
                self.angular_velocity = 2.0 * difference_in_theta
            else:
                self.linear_velocity = 0.5 * distance_to_destination
                self.angular_velocity = 0.5 * difference_in_theta


            twist = self.turtle_twist(self.linear_velocity, self.angular_velocity)
            self.pub_twist.publish(twist)


    def pose_callback(self, pose_msg):
        self.turtle_pose = pose_msg

        if self._state == 1:
            dx = self.turtle_pose.x - self.prev_pose.x
            dy = self.turtle_pose.y - self.prev_pose.y
            dist = math.sqrt(dx * dx + dy * dy)


            self.actual_distance += dist

        self.prev_pose = self.turtle_pose

    async def load_callback(self, request, response):
        """initialize all values that will be used here"""
        self._waypoints = request.waypoint
        self._waypoints.append(self._waypoints[0])
        previous_position = None
        next_position = None
        response.distance = 0
        how_large_x_is = 0.1

        """If there are no waypoints, then exit the callback"""
        if request.waypoint is None:
            return response

        await self.reset.call_async(Empty.Request())

        self.set_pen_msg.off = 1
        await self.set_pen.call_async(self.set_pen_msg)


        for index, position in enumerate(request.waypoint):

            if index > 0:
                next_position = (position.x, position.y)
            else:
                next_position = (position.x, position.y)
                previous_position = (position.x, position.y)

            #corner1_x, corner1_y, corner2_x, corner2_y = position
            corner1_x = position.x + how_large_x_is
            corner1_y = position.y + how_large_x_is
            corner2_x = position.x - how_large_x_is
            corner2_y = position.y - how_large_x_is

            self.teleport_msg.x, self.teleport_msg.y = corner1_x, corner1_y
            await self.teleport.call_async(self.teleport_msg)

            self.set_pen_msg.off = 0
            await self.set_pen.call_async(self.set_pen_msg)

            self.teleport_msg.x, self.teleport_msg.y = corner2_x, corner2_y
            await self.teleport.call_async(self.teleport_msg)
            self.set_pen_msg.off = 1
            await self.set_pen.call_async(self.set_pen_msg)

            ##second set of corners??
            corner1_x = position.x - how_large_x_is
            corner1_y = position.y + how_large_x_is
            corner2_x = position.x + how_large_x_is
            corner2_y = position.y - how_large_x_is

            self.teleport_msg.x, self.teleport_msg.y = corner1_x, corner1_y
            await self.teleport.call_async(self.teleport_msg)

            self.set_pen_msg.off = 0
            await self.set_pen.call_async(self.set_pen_msg)

            self.teleport_msg.x, self.teleport_msg.y = corner2_x, corner2_y
            await self.teleport.call_async(self.teleport_msg)
            self.set_pen_msg.off = 1
            await self.set_pen.call_async(self.set_pen_msg)

            response.distance  += math.dist(previous_position, next_position)
            previous_position = next_position

        self.set_pen_msg = SetPen.Request(r=250,g=0, b=0, width=1)
        await self.set_pen.call_async(self.set_pen_msg)

        self._waypoints_index = 0
        self.planned_distance = response.distance
        return response


    def toggle_callback(self, request, response):
        """ Create a twist suitable for a turtle
            Args:
               request (float) : the forward velocity
               response (float) : the angular velocity

            Returns:
               response - weather the turtle is in a MOVING state or STOPPED state
         """
        if self._state == 1:
            self.get_logger().info('Stopping')
            self._state = 0
        else:
            self.get_logger().info('Issuing Command!')
            self._state = 1
        return response

    def turtle_twist(self, xdot, omega):
        """ Create a twist suitable for a turtle

            Args:
               xdot (float) : the forward velocity
               omega (float) : the angular velocity

            Returns:
               Twist - a 2D twist object corresponding to linear/angular velocity
        """
        return Twist(linear=Vector3(x=xdot, y=0.0, z=0.0),
                     angular=Vector3(x=0.0, y=0.0, z=omega))


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