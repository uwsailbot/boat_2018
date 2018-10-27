
import time

from rosgraph_msgs.msg import Clock

class SimulatorCalculator():
    def __init__(self, rospy, all_data):
        self.all_data = all_data

        self.clock_pub = rospy.Publisher('clock', Clock, queue_size = 1)


    def calc(self):
        pass
        '''
        # Calculate the in-simulator time
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

        if follow_boat:
            camera.x = pos.x
            camera.y = pos.y
        else:	
            # Move camera when mouse is near edge of screen
            # I put this in here so that camera move speed is bound to time and not fps
            camera.x += camera_velocity.x * real_dt
            camera.y += camera_velocity.y * real_dt
        
        if sim_mode is SimMode.DEFAULT:
            # Calculate other things
            tack = calc_tack(heading, wind_heading)
            boom_heading = calc_boom_heading(heading, wind_heading, winch_pos)
            boom_vector = polar_to_rect(1, boom_heading)
            boom_perp_vector = polar_to_rect(1, boom_heading + tack*90)
            app_wind = calc_apparent_wind(wind_heading, boat_speed, heading)
            heading_vector = polar_to_rect(1, heading)
            # components of wind parallel and perpendicular to sail
            wind_par = -proj(app_wind, boom_vector)
            wind_perp = proj(app_wind, boom_perp_vector)
            
            if (wind_perp < 0):
                # Sail is backwinded/luffing
                acc = 0
            else:
                # Calculate drag component (major component when on run)		
                a_perp = 0.03*wind_perp**2
                acc = a_perp * proj(boom_perp_vector, heading_vector)
                # Calculate lift (major component when on reach)
                if wind_par > 0:
                    a_par = 0.03*wind_par**2
                    acc += a_par * proj(boom_perp_vector, heading_vector)
            
            # Wind drag on boat (prominent when in irons)
            acc += 0.01*proj(app_wind, heading_vector)

            # Water drag
            drag = 0.25*boat_speed*abs(boat_speed)
            rudder_drag = 0.2*drag*abs(math.cos(math.radians(rudder_pos)))
            drag += rudder_drag
            boat_speed += (acc - drag)*dt
            
            #old_wind_head = ane_reading
            
            heading -= (90-rudder_pos)*0.4*boat_speed * dt
            heading %= 360
                
            # Update anemometer reading because of new heading and speed
            update_wind()
            
            
            speed_graph[int(ane_reading)%360] = boat_speed
            
            pos.x += math.cos(math.radians(heading)) * boat_speed * dt
            pos.y += math.sin(math.radians(heading)) * boat_speed * dt
            
            # Reset boat if out of bounds
            if abs(pos.x) > 10000 or abs(pos.y) > 10000:
                pos.x = 0
                pos.y = 0
                camera.x = 0
                camera.y = 0
                path = PointArray()
            
            update_vision()
            
            update_gps()
            
            # Don't let drawn path be too long 
            if display_path:
                if len(path.points) > 1000:
                    path = PointArray()
                # Only add current point every half second
                if clock > 0.5 + prev_path_time:
                    pt = Point()
                    pt.x = pos.x
                    pt.y = pos.y
                    path.points.append(pt)
                    prev_path_time = clock
        
        glutPostRedisplay()
        
        if sim_is_running:
            glutTimerFunc(1000/30, calc, 0)
        else:
            rospy.signal_shutdown("Close") 
            glutDestroyWindow(win_ID)
            glutLeaveMainLoop()
        '''