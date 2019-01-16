import time
import math

from std_msgs.msg import Float32
from rosgraph_msgs.msg import Clock
from simulation_data import SimMode
from boat_msgs.msg import Point, PointArray

class SimulatorCalculator():
    def __init__(self, rospy, all_data, camera, ros_publishers):
        self.all_data = all_data
        self.ros_publishers = ros_publishers
        self.clock_pub = rospy.Publisher('clock', Clock, queue_size = 1)
        self.camera = camera


    def calc(self, arg):

        if(self.all_data.last_time == -1):
            last_time = time.time()
        real_dt = time.time() - last_time
        dt = real_dt * self.all_data.speed
        last_time = time.time()
        self.all_data.clock += dt

        if self.all_data.mode_is_default():
            time_msg = Clock()
            time_msg.clock.secs = self.all_data.clock
            time_msg.clock.nsecs = (self.all_data.clock % 1) * (10**9)
            self.clock_pub.publish(time_msg)

        if self.all_data.follow_boat:
            self.camera.x = self.all_data.ros_data["pos"].x
            self.camera.y = self.all_data.ros_data["pos"].y
        else:	
            # Move camera when mouse is near edge of screen
            # I put this in here so that camera move speed is bound to time and not fps
            self.camera.x += self.all_data.ros_data["camera_velocity"].x * real_dt
            self.camera.y += self.all_data.ros_data["camera_velocity"].y * real_dt
        
        if self.all_data.sim_mode is SimMode.DEFAULT:
            # Calculate other things
            tack = self.calc_tack(self.all_data.ros_data["heading"], self.all_data.ros_data["wind_heading"])
            boom_heading = self.calc_boom_heading(self.all_data.ros_data["heading"], self.all_data.ros_data["wind_heading"], self.all_data.ros_data["winch_pos"])
            boom_vector = self.polar_to_rect(1, boom_heading)
            boom_perp_vector = self.polar_to_rect(1, boom_heading + tack*90)
            app_wind = self.calc_apparent_wind(self.all_data.ros_data["wind_heading"], self.all_data.ros_data["boat_speed"], self.all_data.ros_data["heading"])
            heading_vector = self.polar_to_rect(1, self.all_data.ros_data["heading"])
            # components of wind parallel and perpendicular to sail
            wind_par = -self.proj(app_wind, boom_vector)
            wind_perp = self.proj(app_wind, boom_perp_vector)
            
            if (wind_perp < 0):
                # Sail is backwinded/luffing
                acc = 0
            else:
                # Calculate drag component (major component when on run)		
                a_perp = 0.03*wind_perp**2
                acc = a_perp * self.proj(boom_perp_vector, heading_vector)
                # Calculate lift (major component when on reach)
                if wind_par > 0:
                    a_par = 0.03*wind_par**2
                    acc += a_par * self.proj(boom_perp_vector, heading_vector)
            
            # Wind drag on boat (prominent when in irons)
            acc += 0.01*self.proj(app_wind, heading_vector)

            # Water drag
            drag = 0.25*self.all_data.ros_data["boat_speed"]*abs(self.all_data.ros_data["boat_speed"])
            rudder_drag = 0.2*drag*abs(math.cos(math.radians(self.all_data.ros_data["rudder_pos"])))
            drag += rudder_drag
            self.all_data.ros_data["boat_speed"] += (acc - drag)*dt
            
            #old_wind_head = ane_reading
            
            self.all_data.ros_data["heading"] -= (90-self.all_data.ros_data["rudder_pos"])*0.4*self.all_data.ros_data["boat_speed"] * dt
            self.all_data.ros_data["heading"] %= 360
                
            # Update anemometer reading because of new heading and speed
            self.update_wind()
            
            
            self.all_data.speed_graph[int(self.all_data.ros_data["ane_reading"])%360] = self.all_data.ros_data["boat_speed"]
            
            self.all_data.ros_data["pos"].x += math.cos(math.radians(self.all_data.ros_data["heading"])) * self.all_data.ros_data["boat_speed"] * dt
            self.all_data.ros_data["pos"].y += math.sin(math.radians(self.all_data.ros_data["heading"])) * self.all_data.ros_data["boat_speed"] * dt
            
            # Reset boat if out of bounds
            if abs(self.all_data.ros_data["pos"].x) > 10000 or abs(self.all_data.ros_data["pos"].y) > 10000:
                self.all_data.ros_data["pos"].x = 0
                self.all_data.ros_data["pos"].y = 0
                self.camera.x = 0
                self.camera.y = 0
                path = PointArray()
            
            self.update_vision()
            
            self.update_gps()
            
            # Don't let drawn path be too long 
            if display_path:
                if len(path.points) > 1000:
                    path = PointArray()
                # Only add current point every half second
                if clock > 0.5 + prev_path_time:
                    pt = Point()
                    pt.x = self.all_data.ros_data["pos"].x
                    pt.y = self.all_data.ros_data["pos"].y
                    path.points.append(pt)
                    prev_path_time = clock
        
        glutPostRedisplay()
        
        if sim_is_running:
            glutTimerFunc(1000/30, calc, 0)
        else:
            rospy.signal_shutdown("Close") 
            glutDestroyWindow(win_ID)
            glutLeaveMainLoop()

    def calc_apparent_wind(self, true_wind, boat_speed, boat_heading):
        # Use constant wind speed of 8 m/s
        x = self.all_data.ros_data["wind_speed"]*math.cos(math.radians(true_wind))
        x += boat_speed*math.cos(math.radians(boat_heading + 180))
        y = self.all_data.ros_data["wind_speed"]*math.sin(math.radians(true_wind))
        y += boat_speed*math.sin(math.radians(boat_heading + 180))
        return (x, y)


    def calc_direction(self, v):
        angle = math.degrees(math.atan2(v[1], v[0]))
        if angle < 0:
            angle += 360
        return angle


    # returns -1 for port, 1 for starboard
    def calc_tack(self, boat_heading, wind_heading): 
        diff = (self.all_data.ros_data["wind_heading"] - boat_heading)
        if diff > 180:
            diff -= 360
        elif diff < -180:
            diff += 360
        
        # Dead run	
        if diff == 0:
            return 1.0
        return diff/abs(diff)

    # returns heading of vector point from end of boom to mast
    def calc_boom_heading(self, boat_heading, wind_heading, winch):
        winch_range = self.all_data.ros_data["WINCH_MAX"] - self.all_data.ros_data["WINCH_MIN"]
        
        tack = self.calc_tack(boat_heading, self.all_data.ros_data["wind_heading"])
        # Note close-hauled boom is not quite parallel with boat
        return boat_heading - tack * ((self.all_data.ros_data["WINCH_MAX"] - winch) * 75/winch_range + 15)

    # Returns magnitude of projection of u onto v
    def proj(self, u,v):
        v_mag = math.sqrt(v[0]**2 + v[1]**2)
        return (u[0]*v[0] + u[1]*v[1])/v_mag


    # Returns (x,y), given radius and angle in degrees
    def polar_to_rect(self, rad, ang):
        return (rad * math.cos(math.radians(ang)), rad * math.sin(math.radians(ang)))

    def cosd(self, angle):
        return math.cos(math.radians(angle))

    def sind(self, angle):
        return math.sin(math.radians(angle))

    def update_wind(self):
        apparent_wind = self.calc_direction(self.calc_apparent_wind(self.all_data.ros_data["wind_heading"],
                                                                    self.all_data.ros_data["boat_speed"],
                                                                    self.all_data.ros_data["heading"]))
        self.all_data.ros_data["ane_reading"] = (apparent_wind - self.all_data.ros_data["heading"]) % 360
        if self.all_data.ros_data["ane_reading"] < 0:
            self.all_data.ros_data["ane_reading"] = self.all_data.ros_data["ane_reading"] + 360
        
        self.ros_publishers.wind_pub.publish(Float32(self.all_data.ros_data["ane_reading"]))

    def update_vision(self):
        # TODO for obstacles as well        
        vision_points_gps = PointArray()
        for waypoint in self.all_data.ros_data["waypoint_gps"].points:
            if point_is_in_fov(to_lps(waypoint)) and state.challenge is not BoatState.CHA_SEARCH:
                vision_points_gps.points.append(waypoint.pt)
        
        if self.all_data.search_area.target is not None and point_is_in_fov(search_area.target):
            vision_points_gps.points.append(to_gps(search_area.target))
        
        vision_pub.publish(vision_points_gps)

    def update_gps(self, force=False):
        if not force and (time.time() - gps_last_published)*speed < gps_publish_interval:
            return
        gps_last_published = time.time()

        gps = GPS()
        gps.status = GPS.STATUS_FIX
        # simulate position of gps at back of boat
        pos_with_offset = Point(pos.x-POS_OFFSET*cosd(heading), pos.y-POS_OFFSET*sind(heading))
        coords = to_gps(pos_with_offset)
        gps.latitude = coords.y
        gps.longitude = coords.x
        gps.track = (450-heading)%360
        gps.speed = boat_speed * 1.94384 # m/s to KNOTS
        gps_pub.publish(gps)

        orientation = quaternion_from_euler(0,0,math.radians(heading))
        imu = Imu()
        
        # Convertion because they are different types
        imu.orientation.x = orientation[0]
        imu.orientation.y = orientation[1]
        imu.orientation.z = orientation[2]
        imu.orientation.w = orientation[3]
        
        orientation_pub.publish(imu)