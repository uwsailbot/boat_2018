#!/usr/bin/env python
import rospy
import rospkg
import yaml
from boat_msgs.msg import BoatState, Joy, Point, PointArray, Waypoint, WaypointArray
from boat_msgs.srv import ConvertPoint

rospack = ()
state = BoatState()

state_pub = rospy.Publisher('boat_state', BoatState, queue_size=10)
waypoints_pub = rospy.Publisher('waypoints_raw', WaypointArray, queue_size=10)
station_box_pub = rospy.Publisher('bounding_box', PointArray, queue_size=10)
search_area_pub = rospy.Publisher('search_area', PointArray, queue_size=10)
to_gps = rospy.ServiceProxy('lps_to_gps', ConvertPoint)


def state_callback(new_state):
    global state
    state = new_state


def joy_callback(controller):
    global state
    rate = rospy.Rate(100)

    # Use VR nob to switch challenge modes
    if controller.vr < 200 and state.challenge is not BoatState.CHA_STATION:
        state.challenge = BoatState.CHA_STATION
        load_challenge_data()
        state_pub.publish(state)
    elif controller.vr >= 200 and controller.vr < 400 and state.challenge is not BoatState.CHA_LONG:
        state.challenge = BoatState.CHA_LONG
        load_challenge_data()
        state_pub.publish(state)
    elif controller.vr >= 400 and controller.vr < 600 and state.challenge is not BoatState.CHA_NAV:
        state.challenge = BoatState.CHA_NAV
        load_challenge_data()
        state_pub.publish(state)
    elif controller.vr >= 600 and controller.vr < 800 and state.challenge is not BoatState.CHA_SEARCH:
        state.challenge = BoatState.CHA_SEARCH
        load_challenge_data()
        state_pub.publish(state)
    elif controller.vr >= 800 and state.challenge is not BoatState.CHA_AVOID:
        state.challenge = BoatState.CHA_AVOID
        load_challenge_data()
        state_pub.publish(state)

    # If switch A is in the up position, set auto
    if controller.switch_a == Joy.SWITCH_UP and state.major is not BoatState.MAJ_AUTONOMOUS:
        state.major = BoatState.MAJ_AUTONOMOUS
        state.minor = BoatState.MIN_INITIALIZE
        state_pub.publish(state)

    # If switch A is in the neutral position, set rc
    elif controller.switch_a == Joy.SWITCH_MIDDLE and state.major is not BoatState.MAJ_RC:
        state.major = BoatState.MAJ_RC
        state.minor = BoatState.MIN_COMPLETE
        state_pub.publish(state)

    # If switch A is in the down position, set disabled
    elif controller.switch_a == Joy.SWITCH_DOWN and state.major is not BoatState.MAJ_DISABLED:
        state.major = BoatState.MAJ_DISABLED
        state.minor = BoatState.MIN_COMPLETE
        state_pub.publish(state)

    rate.sleep()


def load_challenge_data():

    # Search challenge
    if state.challenge is BoatState.CHA_SEARCH:
        data = yaml.load(open(rospack.get_path('boat_interfaces') + "/challenges/search.yaml"))

        center = Point(data["Center"]["x"], data["Center"]["y"])
        edge = Point(center.x + data["radius"], center.y)

        if data["Center"]["lps"]:
            center = to_gps(center).pt
            edge = to_gps(edge).pt

        #TODO: Load target pt

        search_area_pub.publish([center, edge])

    # Station keeping challenge
    elif state.challenge is BoatState.CHA_STATION:
        data = yaml.load(open(rospack.get_path('boat_interfaces') + "/challenges/station.yaml"))

        box = PointArray()

        for pt in data["Box"]:
            cur_pt = Point(pt["x"], pt["y"])
            if pt["lps"]:
                cur_pt = to_gps(cur_pt).pt
            box.points.append(cur_pt)

        station_box_pub.publish(box)

    # Navigation and Long-Distance challenges
    elif state.challenge is BoatState.CHA_LONG or state.challenge is BoatState.CHA_NAV:
        if state.challenge is BoatState.CHA_LONG:
            data = yaml.load(open(rospack.get_path('boat_interfaces') + "/challenges/long.yaml"))
        elif state.challenge is BoatState.CHA_NAV:
            data = yaml.load(open(rospack.get_path('boat_interfaces') + "/challenges/nav.yaml"))

        path = WaypointArray()

        for pt in data["Path"]:
            cur_pt = Point(pt["x"], pt["y"])
            if pt["lps"]:
                cur_pt = to_gps(cur_pt).pt

            cur_waypt = Waypoint(cur_pt, Waypoint.TYPE_ROUND)
            if not pt["round"]:
                cur_waypt.type = Waypoint.TYPE_INTERSECT

            path.points.append(cur_waypt)

        waypoints_pub.publish(path)


def listener():
    global rospack
    rospack = rospkg.RosPack()

    # Setup subscribers
    rospy.init_node('boat_state_controller_node')
    rospy.Subscriber('joy', Joy, joy_callback)
    rospy.Subscriber('boat_state', BoatState, state_callback)
    rospy.spin()


if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
