from OpenGL.GLUT import *
from OpenGL.GL import *

from simulation_data import SimMode

from boat_msgs.msg import Point, PointArray, BoatState, Waypoint, WaypointArray, Joy


class OpenGLHMI():
    def __init__(self, display_data, all_data, ros_interfaces):
        self.display_data = display_data
        self.all_data = all_data
        self.camera = self.display_data.camera
        self.ros_services = ros_interfaces.services
        self.ros_publishers = ros_interfaces.publishers

        self.camera_velocity = Point(0,0)

        self.cur_slider = () # Try this

        self.sliders = None

    def set_sliders(self, sliders):
        self.sliders = sliders

    # Window resize callback
    def resize(self, width, height):
        self.display_data.win_width = width
        self.display_data.win_height = height

        glutInitWindowSize(self.display_data.win_width, self.display_data.win_height)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        glOrtho(0.0, self.display_data.win_width, 0.0, self.display_data.win_height, -1, 1)
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        self.camera.resize(self.display_data.win_width, self.display_data.win_height)


    # Handler for mouse presses
    def mouse_handler(self, button, mouse_state, x, y):
        if mouse_state != GLUT_DOWN:
            self.cur_slider = ()
            return

        # If status panels are being clicked on
        if (x <= 180 and self.display_data.show_details) or (x >= self.display_data.win_width - 120) and button == GLUT_LEFT_BUTTON:
            for key in self.sliders:
                if self.sliders[key].contains(x,y, self.display_data.win_height):
                    self.cur_slider = self.sliders[key]
                    self.cur_slider.handle_mouse(x,y)
            return

        if button == GLUT_RIGHT_BUTTON:
            if self.all_data.sim_mode is SimMode.DEFAULT or self.all_data.sim_mode is SimMode.CONTROLLER:
                self.all_data.ros_data["waypoint_gps"] = WaypointArray()
                gps_bounding_box = PointArray()
                gps_search_area = PointArray()
                search_area.target = None
                self.ros_publishers.waypoint_pub.publish(self.all_data.ros_data["waypoint_gps"])
                self.ros_publishers.square_pub.publish(gps_bounding_box)
                self.ros_publishers.search_area_pub.publish(gps_search_area)

        elif self.cur_slider is () and (self.all_data.sim_mode is SimMode.DEFAULT or self.all_data.sim_mode is SimMode.CONTROLLER) and self.all_data.ros_data["state"].challenge is not BoatState.CHA_STATION and self.all_data.ros_data["state"].challenge is not BoatState.CHA_SEARCH and (button == GLUT_LEFT_BUTTON or button == GLUT_MIDDLE_BUTTON):
            newPt = Point()
            (lps_x,lps_y) = self.camera.screen_to_lps(x,y)
            newPt.x = lps_x
            newPt.y = lps_y
            if button == GLUT_LEFT_BUTTON:
                coords = Waypoint(self.ros_services.to_gps(newPt), Waypoint.TYPE_INTERSECT)
            elif button == GLUT_MIDDLE_BUTTON:
                coords = Waypoint(self.ros_services.to_gps(newPt), Waypoint.TYPE_ROUND)
            self.all_data.ros_data["waypoint_gps"].points.append(coords)
            
            self.ros_publishers.waypoint_pub.publish(self.all_data.ros_data["waypoint_gps"])

        elif self.cur_slider is () and (self.all_data.sim_mode is SimMode.DEFAULT or self.all_data.sim_mode is SimMode.CONTROLLER) and self.all_data.ros_data["state"].challenge is BoatState.CHA_STATION and button == GLUT_LEFT_BUTTON:
            newPt = Point()
            (lps_x,lps_y) = self.camera.screen_to_lps(x,y)
            newPt.x = lps_x
            newPt.y = lps_y
            coords = to_gps(newPt)
            
            for p in gps_bounding_box.points:
                l = to_lps(p)
                if math.hypot(l.y - newPt.y, l.x-newPt.x) < 15:
                    print "Distance between buoys is too small, must be at least 15m"
                    return
            # Reset if we were gonna add to a list of 4 points already
            if len(gps_bounding_box.points) == 4:
                gps_bounding_box = PointArray()
            
            gps_bounding_box.points.append(coords)
            self.ros_publishers.square_pub.publish(gps_bounding_box)
            
            self.ros_publishers.waypoint_pub.publish(self.all_data.ros_data["waypoint_gps"])

        elif self.cur_slider is () and (self.all_data.sim_mode is SimMode.DEFAULT or self.all_data.sim_mode is SimMode.CONTROLLER) and state.challenge is BoatState.CHA_SEARCH and button == GLUT_LEFT_BUTTON:
            (lps_x,lps_y) = self.camera.screen_to_lps(x,y)		
            new_point = to_gps(Point(lps_x, lps_y))

            if len(gps_search_area.points) == 2:
                if search_area.target is None:
                    # set search target
                    # no other nodes need to know this since this is for testing only,
                    # so we can just do it here and not publish anything 
                    search_area.target = to_lps(new_point) #TODO fix this when lps origin is shifted.
                    search_area.setup_coverage()
                else:
                    # Reset if we were gonna add to a list of 2 points and a target already
                    search_area.target = None
                    gps_search_area = PointArray()
                    gps_search_area.points.append(new_point)
            else:
                gps_search_area.points.append(new_point)
            
            self.ros_publishers.search_area_pub.publish(gps_search_area)

        elif (button == 3 or button == 4) and mouse_state == GLUT_DOWN:
            
            if not follow_boat:
                # used to keep mouse pointing at same lps position when zooming
                (mouse_x, mouse_y) = self.camera.screen_to_lps(x,y)
            
            if button == 3:
                self.camera.scale *= 1.04
                if self.camera.scale > 100:
                    self.camera.scale = 100
            else:
                self.camera.scale *= 0.96
                if self.camera.scale < 0.1:
                    self.camera.scale = 0.1
            
            if not follow_boat:
                # keep mouse pointing at same lps position when zooming
                (new_mouse_x, new_mouse_y) = self.camera.screen_to_lps(x,y)
                self.camera.x -= new_mouse_x - mouse_x
                self.camera.y -= new_mouse_y - mouse_y

    # Handler for mouse position
    def passive_mouse_handler(self, x,y):
        #save mouse pos
        mouse_pos = Point(x,y)

        # if mouse is on right info panel, don't move self.camera
        if x > self.display_data.win_width-120:
            self.camera_velocity.x = 0
            self.camera_velocity.y = 0
            return

        camera_move_speed = self.all_data.ros_data["camera_move_speed"]
        
        # set self.camera_velocity when mouse is near edge of screen or edge of info display panel
        if (x < 55 and x > 5 and not self.display_data.show_details) or (x < 180+50 and x > 180 and self.display_data.show_details):
            self.camera_velocity.x = -camera_move_speed / self.camera.scale
        elif x > self.display_data.win_width-120-50:
            self.camera_velocity.x = camera_move_speed / self.camera.scale
        else:
            self.all_data.ros_data["camera_velocity"].x = 0
        # The > 5 makes sure that when we scroll off the window, the self.camera stops moving
        if y < 55 and y > 5:
            self.camera_velocity.y = camera_move_speed / self.camera.scale
        elif y > self.display_data.win_height-55 and y < self.display_data.win_height-5:
            self.camera_velocity.y = -camera_move_speed / self.camera.scale
        else:
            self.camera_velocity.y = 0	

    # Handler for mouse dragging
    def motion_handler(self, x,y):
        if self.cur_slider is not ():
            self.cur_slider.handle_mouse(x,y)


    # Handler for all key presses that cannot be represented by an ASCII code
    def keyboard_handler(self, key, mousex, mousey):
        global joy
        if key == GLUT_KEY_LEFT and self.all_data.should_sim_joy:
            self.all_data.joy.right_stick_x -= 20
            if self.all_data.joy.right_stick_x < 0:
                self.all_data.joy.right_stick_x = 0
            self.ros_publishers.joy_pub.publish(self.all_data.joy)
        elif key == GLUT_KEY_RIGHT and self.all_data.should_sim_joy:
            self.all_data.joy.right_stick_x += 20
            if self.all_data.joy.right_stick_x > Joy.JOY_RANGE:
                self.all_data.joy.right_stick_x = Joy.JOY_RANGE
            self.ros_publishers.joy_pub.publish(self.all_data.joy)
        elif key == GLUT_KEY_UP and self.all_data.should_sim_joy:
            self.all_data.joy.left_stick_y += 50
            if self.all_data.joy.left_stick_y > Joy.JOY_RANGE:
                self.all_data.joy.left_stick_y = Joy.JOY_RANGE
            self.ros_publishers.joy_pub.publish(self.all_data.joy)
        elif key == GLUT_KEY_DOWN and self.all_data.should_sim_joy:
            self.all_data.joy.left_stick_y -= 50
            if self.all_data.joy.left_stick_y < 0:
                self.all_data.joy.left_stick_y = 0
            self.ros_publishers.joy_pub.publish(self.all_data.joy)


    # Handler for all key presses that can be represented by an ASCII code
    def ASCII_handler(self, key, mousex, mousey):
        # Handle cheat codes
        self.all_data.cur_input += key;
        valid = False
        for code in self.display_data.boat_imgs:
            if code == self.all_data.cur_input:
                cur_boat_img = self.display_data.boat_imgs[code]
                cur_rudder_img = self.display_data.rudder_imgs[code]
                cur_sail_img = self.display_data.sail_imgs[code]
                valid = False
                break
            if code.startswith(self.all_data.cur_input):
                valid = True
        if not valid:
            self.all_data.cur_input = ""
        
        if key is chr(27):
            sim_is_running = False
        elif key is 'p':
            self.pause_sim()
        elif key is 'm':
            if self.all_data.sim_mode is SimMode.DEFAULT:
                self.all_data.sim_mode = SimMode.REPLAY
                reset_origin_on_next_gps = True
            elif self.all_data.sim_mode is SimMode.REPLAY:
                self.all_data.sim_mode = SimMode.CONTROLLER
                reset_origin_on_next_gps = True
            else:
                self.all_data.sim_mode = SimMode.DEFAULT
                reset_origin_on_next_gps = False
            print 'Changed sim mode, is now', self.all_data.sim_mode
        elif key is 'i':
            self.display_data.show_details = not self.display_data.show_details
        elif key is 'c':
            self.all_data.ros_data["path"] = PointArray()
            self.display_data.display_path = not self.display_data.display_path
        elif key is 'y':
            self.all_data.follow_boat = not self.all_data.follow_boat
        elif key is '0':
            sound = not sound
        
        elif key is 'x' and (self.all_data.sim_mode is SimMode.DEFAULT or self.all_data.sim_mode is SimMode.CONTROLLER):
            (lps_x,lps_y) = self.camera.screen_to_lps(mousex,mousey)
            coords = Waypoint(self.ros_services.to_gps(Point(lps_x, lps_y)), Waypoint.TYPE_ROUND)
            self.all_data.ros_data["waypoint_gps"].points.append(coords)
            self.ros_publishers.waypoint_pub.publish(self.all_data.ros_data["waypoint_gps"])
        
        if self.all_data.sim_mode is SimMode.DEFAULT or self.all_data.sim_mode is SimMode.CONTROLLER:
            if key is '1' and self.all_data.should_sim_joy:
                self.all_data.joy.switch_a = Joy.SWITCH_MIDDLE
                self.ros_publishers.joy_pub.publish(self.all_data.joy)
            elif key is '2' and self.all_data.should_sim_joy:
                self.all_data.joy.switch_a = Joy.SWITCH_UP
                self.ros_publishers.joy_pub.publish(self.all_data.joy)
            elif key is '3' and self.all_data.should_sim_joy:
                self.all_data.joy.switch_a = Joy.SWITCH_DOWN
                self.ros_publishers.joy_pub.publish(self.all_data.joy)
            elif key is '4' and self.all_data.should_sim_joy:
                self.all_data.joy.vr = max(joy.vr - 200, 0)
                self.ros_publishers.joy_pub.publish(self.all_data.joy)
            elif key is '5' and self.all_data.should_sim_joy:
                self.all_data.joy.vr = min(joy.vr + 200, 1000-1)
                self.ros_publishers.joy_pub.publish(self.all_data.joy)
            elif key is 'q':
                self.all_data.joy.right_stick_x = Joy.JOY_RANGE/2.0
                self.ros_publishers.joy_pub.publish(self.all_data.joy)
            elif key is 'a':
                self.all_data.ros_data["wind_heading"] += 5
                self.all_data.ros_data["wind_heading"] = self.all_data.ros_data["wind_heading"] % 360
            elif key is 'd':
                self.all_data.ros_data["wind_heading"] -= 5
                if self.all_data.ros_data["wind_heading"] < 0:
                    self.all_data.ros_data["wind_heading"] += 360
            elif key is ' ':
                pos.x = 0
                pos.y = 0
                self.camera.x = 0
                self.camera.y = 0
                self.camera.scale = 10
                path = PointArray()
                update_gps(True)
                

    def wind_speed_slider_callback(self, value):
        self.all_data.wind_speed = value

    def sim_speed_slider_callback(self, value):
        self.all_data.speed = value / 100.0


    def pause_sim(self):

        self.all_data.pause = not self.all_data.pause
        if self.all_data.pause is True:
            self.all_data.pre_pause_speed = self.all_data.speed
            self.all_data.speed = 0
        else:
            self.all_data.speed = self.all_data.pre_pause_speed
        self.sliders["Sim speed"].change_val(self.all_data.speed*100)
