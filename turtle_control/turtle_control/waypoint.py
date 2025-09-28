"""A demonstration Node for ME495."""
import math

from example_interfaces.msg import Int64
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from std_srvs.srv import Empty
from turtlesim_msgs.srv import SetPen, TeleportAbsolute
from turtlesim_msgs.msg import Pose
from turtle_interfaces.srv import Waypoints
from geometry_msgs.msg import Twist, Vector3
from turtle_interfaces.msg import ErrorMetric


class WayPoint(Node):
    """
    Waypoint Node interacts with the turtle sim

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

        #personal call back group to avoid blocking
        self.personal_call_back_group = MutuallyExclusiveCallbackGroup()

        #state determines if toggle is on or off, 0=off, 1=on
        self._state = 0


        self.velocity = 10.0
        self.turtle_pose = Pose()

        self.turtle_live_listener = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)

        self._waypoints = None

        self._pub = self.create_publisher(Int64, 'count', 10)
        self._tmr = self.create_timer(0.011, self.timer_callback)
        self._tog = self.create_service(Empty, 'toggle', self.toggle_callback)

        self.reset = self.create_client(Empty, '/reset', callback_group=self.personal_call_back_group)
        self.set_pen = self.create_client(SetPen, 'turtle1/set_pen', callback_group=self.personal_call_back_group)
        self.teleport = self.create_client(TeleportAbsolute, 'turtle1/teleport_absolute', callback_group=self.personal_call_back_group)

        self.pub_twist = self.create_publisher(Twist, "turtle1/cmd_vel", 10)
        self.pub_metrics = self.create_publisher(ErrorMetric, "/loop_metrics", 10)


        self.set_pen_msg = SetPen.Request(r=100,g=100, b=100, width=5)
        self.teleport_msg = TeleportAbsolute.Request()


        self._load = self.create_service(Waypoints, 'load', self.load_callback)
        self._sub = self.create_subscription(Int64, 'uncount', self.uncount_callback, 10)
        self._count = 0
        self.tolerance = 0.05

        self.complete_loops = 0
        self.actual_distance = 0.0
        self.prev_pose = None
        self.planned_distance = 0.0


    def timer_callback(self):
        """Increment the count at a fixed frequency."""
        if self._state == 1:

            if self.turtle_pose is None:
                self.get_logger().info("POSE no recieved yet")
                return

            dest =  self._waypoints[self._waypoints_index]
            destination_x = dest.x
            destination_y = dest.y
            destination_z = dest.z
            #destination_x, destination_y, destination_z =  self._waypoints[self._waypoints_index]

            current_x = self.turtle_pose.x
            current_y = self.turtle_pose.y
            current_theta = self.turtle_pose.theta
            #current_x, current_y, current_theta = self.turtle_pose

            dy = destination_y - current_y
            dx = destination_x - current_x

            distance_to_destination = math.sqrt(dx * dx + dy * dy)
            difference_in_theta = math.atan2(dy, dx) - current_theta

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
                    self._state = 0

                    self.complete_loops = self.complete_loops + 1
                    self.error = self.actual_distance - self.planned_distance*self.complete_loops

                    self.pub_metrics.publish(ErrorMetric(complete_loops=int(self.complete_loops), actual_distance=float(self.actual_distance),
                                                         error=float(self.error)))

                    self.get_logger().info(f"[loop {self.complete_loops} complete] planned_distance={self.planned_distance}"
                                           f" actual_distance={self.actual_distance} error={self.error}")
                    return


            kp_angular_velocity = 0.5
            kp_linear_velocity = 0.5

            self.linear_velocity = kp_linear_velocity * distance_to_destination
            self.angular_velocity = kp_angular_velocity * difference_in_theta

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


    def angle_between(self, p1, p2):
        # Calculate the components of the vector
        dx = p2.x - p1.x
        dy = p2.y - p1.y

        # Use atan2 to get the angle in radians, then convert to degrees
        angle_radians = math.atan2(dy, dx)
        angle_degrees = math.degrees(angle_radians)

        #return the angle
        return angle_degrees


    async def load_callback(self, request, response):
        self.get_logger().info('load_callback  called')

        self._waypoints = request.waypoint
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

        self.set_pen_msg = SetPen.Request(r=250,g=0, b=0, width=1)
        await self.set_pen.call_async(self.set_pen_msg)

        self._waypoints_index = 0
        self.planned_distance = response.distance
        return response



    def toggle_callback(self, request, response):
        if self._state == 1:
            self.get_logger().info('Stopping')
            self._state = 0
        else:
            self.get_logger().info('Issuing Command!')
            self._state = 1
        return response



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