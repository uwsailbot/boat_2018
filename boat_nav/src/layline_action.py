#!/usr/bin/env python
import rospy
from actionlib import SimpleActionClient, SimpleActionServer
from boat_msgs.msg import LaylineAction as LaylineActionMsg, LaylineFeedback, LaylineResult, BoatState, Point, Waypoint, TackingAction, TackingGoal, GPS
from std_msgs.msg import Float32
from actionlib_msgs.msg import GoalStatus
from boat_utilities import angles, units


class LaylineAction(object):
    # create messages that are used to publish feedback/result
    _feedback = LaylineFeedback()
    _result = LaylineResult()

    def __init__(self, name):
        self._name = name
        self._as = SimpleActionServer(self._name,
                                      LaylineActionMsg,
                                      execute_cb=self.layline_callback,
                                      auto_start=False)
        self._as.start()
        self.tacking_client = SimpleActionClient('tacking_action', TackingAction)
        self.cur_pos = Point()
        self.min_speed = rospy.get_param('/boat/nav/min_tacking_speed')
        self.layline = rospy.get_param('/boat/nav/layline')
        self.target_heading = 0
        self.new_target = False
        self.boat_speed = 0
        self.ane_reading = 0
        self.apparent_wind_heading = 0
        self.wind_coming = 0
        self.compass = 0
        self.sub_ane = rospy.Subscriber('anemometer', Float32, self.anemometer_callback)
        self.target_heading_sub = rospy.Subscriber('target_heading', Float32,
                                                   self.target_heading_callback)
        self.target_sub = rospy.Subscriber('target_point', Waypoint, self.target_callback)
        self.sub_heading = rospy.Subscriber('compass', Float32, self.compass_callback)
        self.target_pub = rospy.Publisher('target_heading', Float32, queue_size=10)
        self.pos_sub = rospy.Subscriber('lps', Point, self.position_callback)
        self.gps_sub = rospy.Subscriber('gps_raw', GPS, self.gps_callback)
        self.rate = rospy.Rate(100)
        self.tacking_client.wait_for_server()

    def target_heading_callback(self, new_target_heading):
        self.target_heading = new_target_heading.data

    def position_callback(self, position):
        self.cur_pos = position

    def target_callback(self, new_target):
        self.new_target = True
        #print "new target"

    def gps_callback(self, gps):
        self.boat_speed = units.from_knots(gps.speed)

    def anemometer_callback(self, new_heading):
        self.ane_reading = new_heading.data
        self.update_apparent_wind()

    def compass_callback(self, compass):
        self.compass = compass.data
        self.update_apparent_wind()

    def update_apparent_wind(self):
        self.apparent_wind_heading = angles.normalize(self.ane_reading + self.compass)
        self.wind_coming = angles.opposite(self.apparent_wind_heading)

    def layline_callback(self, goal):
        # helper variables
        success = False
        preempted = False
        did_hit_midpoint = False
        self.new_target = False

        # publish info to the console for the user
        self._feedback.status = " Entered Layline Action Callback. "
        rospy.loginfo(rospy.get_caller_id() + self._feedback.status)

        pos = self.cur_pos
        tar = goal.target.pt

        a = angles.tand(self.compass)
        b = -1
        c = tar.y - a * tar.x
        dx = pos.x - tar.x
        dy = pos.y - tar.y

        d_perp = abs(a * pos.x + b * pos.y + c) / (a * a + b * b)**0.5
        d_par = (dx * dx + dy * dy - d_perp * d_perp)**0.5

        if angles.is_on_left(goal.alt_tack_angle, self.wind_coming):
            tacking_direction = 1
        else:
            tacking_direction = -1
        self._feedback.status = " Tacking away from mark to hit layline. "
        rospy.loginfo(rospy.get_caller_id() + self._feedback.status)
        new_target = angles.normalize(self.wind_coming - tacking_direction * self.layline)
        self.target_heading = new_target
        self.target_pub.publish(Float32(self.target_heading))

        tacking_goal = TackingGoal(direction=tacking_direction)
        self.tacking_client.send_goal(tacking_goal)

        endtime = rospy.Time.now() + rospy.Duration(10)

        # TODO: Sometimes this loop abruptly exits for no reason
        while (self.tacking_client.get_state() is GoalStatus.ACTIVE or\
            self.tacking_client.get_state() is GoalStatus.PENDING) and rospy.Time.now() < endtime and not did_hit_midpoint:

            # If we reach approximately half of the perpendicular distance, we know we must cancel the layline action
            # and return to our original tack. We use 48% rather than 50% here since the boat cannot respond instantly
            # due to inertia, etc. So this provides us a bit of buffer to get back on track
            # TODO: Tune this constant
            pos = self.cur_pos
            cur_d_perp = abs(a * pos.x + b * pos.y + c) / (a * a + b * b)**0.5
            if cur_d_perp < d_perp * 0.48:
                did_hit_midpoint = True

        # If we hit the midpoint, cancel the current tack and return to the original heading
        if did_hit_midpoint:
            self.tacking_client.cancel_goal()

            self._feedback.status = " Reached 48% perpendicular distance, returning to original heading."
            rospy.loginfo(rospy.get_caller_id() + self._feedback.status)

            tacking_goal.direction = tacking_goal.direction * -1
            self.tacking_client.send_goal(tacking_goal)
            if not self.tacking_client.wait_for_result(rospy.Duration(10)):
                self.tacking_client.cancel_goal()
                self._result.success = False
                self._result.target_heading = self.target_heading
                self._feedback.status = " Second tacking goal expired"
                rospy.loginfo(rospy.get_caller_id() + self._feedback.status)
                self._as.set_succeeded(self._result)
                return

            self._result.success = True
            self._result.target_heading = self.target_heading
            self._feedback.status = " Completed shortened layline action"
            rospy.loginfo(rospy.get_caller_id() + self._feedback.status)
            self._as.set_succeeded(self._result)
            return

        # If we hit this state, the tack failed. Attempt to return to the original heading
        if not self.tacking_client.get_result() or not self.tacking_client.get_result().success:
            self.tacking_client.cancel_goal()
            tacking_goal.direction = tacking_goal.direction * -1
            self.tacking_client.send_goal(tacking_goal)
            if not self.tacking_client.wait_for_result(rospy.Duration(10)):
                self.tacking_client.cancel_goal()
            self._result.success = success
            self._result.target_heading = self.target_heading
            self._feedback.status = " Tacking goal expired"
            rospy.loginfo(rospy.get_caller_id() + self._feedback.status)
            self._as.set_succeeded(self._result)
            return

        self.target_heading = self.tacking_client.get_result().target_heading
        hit_layline = False
        self._feedback.status = " Waiting to hit layline. "
        rospy.loginfo(rospy.get_caller_id() + self._feedback.status)

        # Wait until we hit the layline heading
        while (not hit_layline or self.boat_speed < self.min_speed) and not preempted:
            direct_heading = angles.atan2d(goal.target.pt.y - self.cur_pos.y,
                                           goal.target.pt.x - self.cur_pos.x)
            direct_heading = angles.normalize(direct_heading)

            lower_bound = angles.normalize(goal.alt_tack_angle - goal.overshoot_angle)
            upper_bound = angles.normalize(goal.alt_tack_angle + goal.overshoot_angle)

            if (tacking_direction is 1 and angles.is_on_left(direct_heading, upper_bound)) or\
             (tacking_direction is -1 and angles.is_on_right(direct_heading, lower_bound)):
                hit_layline = True

            if self._as.is_preempt_requested() or self.new_target:
                self._result.success = success
                self._result.target_heading = self.target_heading
                self._feedback.status = " Preempted"
                rospy.loginfo(rospy.get_caller_id() + self._feedback.status)
                self._as.set_preempted()
                preempted = True
            self.rate.sleep()

        # If preempted in the loop, exit the action
        if preempted:
            return

        self._feedback.status = " Hit layline. Tacking towards mark"
        rospy.loginfo(rospy.get_caller_id() + self._feedback.status)
        # Reverse tacking direction
        tacking_goal.direction = tacking_direction * -1
        self.target_heading = goal.alt_tack_angle
        self.target_pub.publish(Float32(self.target_heading))
        self.tacking_client.send_goal(tacking_goal)

        # Adjust time delay until the tack is considered failed, and we return to planning
        if not self.tacking_client.wait_for_result(rospy.Duration(10)):
            self.tacking_client.cancel_goal()
            tacking_goal.direction = tacking_goal.direction * -1
            self.tacking_client.send_goal(tacking_goal)
            if not self.tacking_client.wait_for_result(rospy.Duration(10)):
                self.tacking_client.cancel_goal()
            self._result.success = success
            self._result.target_heading = self.target_heading
            self._feedback.status = " Second tacking goal expired"
            rospy.loginfo(rospy.get_caller_id() + self._feedback.status)
            return

        self.target_heading = self.tacking_client.get_result().target_heading
        success = True
        self._result.success = success
        self._result.target_heading = self.target_heading
        self._feedback.status = " Completed Layline Action. "
        rospy.loginfo(rospy.get_caller_id() + self._feedback.status)
        self._as.set_succeeded(self._result)


if __name__ == '__main__':
    rospy.init_node('layline_action')
    server = LaylineAction(rospy.get_name())
    rospy.spin()
