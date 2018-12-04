#!/usr/bin/env python
import rospy
from actionlib import SimpleActionServer
from boat_msgs.msg import MaxVMGAction as MaxVMGActionMsg, MaxVMGFeedback, MaxVMGResult, GPS, Point
from std_msgs.msg import Bool, Float32
from boat_utilities import angles


class MaxVMGAction(object):

    # create messages that are used to publish feedback/result
    _feedback = MaxVMGFeedback()
    _result = MaxVMGResult()

    def __init__(self, name):
        self._name = name
        self._as = SimpleActionServer(
            self._name, MaxVMGActionMsg, execute_cb=self.execute, auto_start=False)
        self._as.start()

        self.layline = rospy.get_param('/boat/nav/layline')

        self.ane_reading = 0
        self.heading = 0
        self.speed = 0
        self.pos = Point()
        self.max_vmg = 0

        # Subscribers
        rospy.Subscriber('anemometer', Float32, self.anemometer_callback)
        rospy.Subscriber('lps', Point, self.pos_callback)
        rospy.Subscriber('compass', Float32, self.heading_callback)
        rospy.Subscriber('gps_raw', GPS, self.speed_callback)

        # Publishers
        self.target_heading_pub = rospy.Publisher('rudder_pid/setpoint', Float32, queue_size=10)
        self.pid_enable_pub = rospy.Publisher('rudder_pid/enable', Bool, queue_size=10)
        #self.state_pub = rospy.Publisher('boat_state', BoatState, queue_size=10)

        self.rate = rospy.Rate(100)

    def speed_callback(self, gps):
        self.speed = gps.speed

    def heading_callback(self, compass):
        self.heading = compass.data

    def pos_callback(self, pos):
        self.pos = pos

    def anemometer_callback(self, anemometer):
        self.ane_reading = anemometer.data

    def vmg(self):
        return angles.cosd(self.heading - self.direct_heading()) * self.speed

    def direct_heading(self):
        orig = self.pos
        dest = self.goal.target
        return angles.atan2d(dest.y - orig.y, dest.x - orig.x)

    def execute(self, goal):
        self.goal = goal

        success = False
        preempted = False

        self._feedback.status = "Entered Action Callback"

        # Publish info to the console for the user
        if goal.direction is 1:
            rospy.loginfo('Max VMG Action: Startup. Finding VMG on right of target')
        elif goal.direction is -1:
            rospy.loginfo('Max VMG Action: Startup. Finding VMG on left of target')
        else:
            rospy.loginfo('Max VMG Action: Illegal Arguments')

        # Set state and stuff if needed

        # Make sure PID is enabled
        self.target_heading_pub.publish(Float32(self.heading))
        self.pid_enable_pub.publish(Bool(True))

        # If we're pointing towards the wrong side of the target, start by pointing towards the target
        if (goal.direction is 1 and self.heading < self.direct_heading())\
         or (goal.direction is -1 and self.heading > self.direct_heading()):
            self.target_heading_pub.publish(self.direct_heading())

        # Start executing the action
        while not success and not preempted:

            # If we are trying to find the max VMG to the right of the target and we're pointing on the correct side of the target:
            if goal.direction is 1 and self.heading > self.direct_heading() - 2.5:

                cur_vmg = self.vmg()

                # Keep turning further right until we find the max vmg
                if cur_vmg > self.max_vmg:
                    self.target_heading_pub.publish(self.heading + 2)
                    self.max_vmg = cur_vmg
                else:
                    success = True

            # If we are trying to find the max VMG to the left of the target and we're pointing on the correct side of the target:
            elif goal.direction is -1 and self.heading < self.direct_heading() + 2.5:

                cur_vmg = self.vmg()

                # Keep turning further left until we find the max vmg
                if cur_vmg > self.max_vmg:
                    self.target_heading_pub.publish(self.heading - 2)
                    self.max_vmg = cur_vmg
                else:
                    success = True

            # If the action was preempted...
            if self._as.is_preempt_requested():
                rospy.loginfo('Max VMG Action: Preempted')
                # Cleanup
                self._result.success = False
                self._feedback.status = "Preempted"
                self._as.set_preempted()
                preempted = True

            # Sleep
            self.rate.sleep()

        # Once we're outside of the loop, if we succeeded...
        if success:
            self._result.success = success
            rospy.loginfo('Max VMG Action: Success')
            self._as.set_succeeded(self._result)


if __name__ == '__main__':
    rospy.init_node('max_vmg_action')
    server = MaxVMGAction(rospy.get_name())
    rospy.spin()
