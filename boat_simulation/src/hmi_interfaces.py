from OpenGL.GLUT import *
from OpenGL.GL import *

from boat_msgs.msg import Point, PointArray


class OpenGLHMI():
    def __init__(self, display_data, all_data):
        self.display_data = display_data
        self.all_data = all_data
        self.camera = self.display_data.camera

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
            cur_slider = ()
            return

        # If status panels are being clicked on
        if (x <= 180 and show_details) or (x >= win_width - 120) and button == GLUT_LEFT_BUTTON:
            for key in sliders:
                if sliders[key].contains(x,y, win_height):
                    cur_slider = sliders[key]
                    cur_slider.handle_mouse(x,y)
            return

        if button == GLUT_RIGHT_BUTTON:
            if sim_mode is SimMode.DEFAULT or sim_mode is SimMode.CONTROLLER:
                waypoint_gps = WaypointArray()
                gps_bounding_box = PointArray()
                gps_search_area = PointArray()
                search_area.target = None
                waypoint_pub.publish(waypoint_gps)
                square_pub.publish(gps_bounding_box)
                search_area_pub.publish(gps_search_area)

        elif cur_slider is () and (sim_mode is SimMode.DEFAULT or sim_mode is SimMode.CONTROLLER) and state.challenge is not BoatState.CHA_STATION and state.challenge is not BoatState.CHA_SEARCH and (button == GLUT_LEFT_BUTTON or button == GLUT_MIDDLE_BUTTON):
            newPt = Point()
            (lps_x,lps_y) = self.camera.screen_to_lps(x,y)
            newPt.x = lps_x
            newPt.y = lps_y
            if button == GLUT_LEFT_BUTTON:
                coords = Waypoint(to_gps(newPt), Waypoint.TYPE_INTERSECT)
            elif button == GLUT_MIDDLE_BUTTON:
                coords = Waypoint(to_gps(newPt), Waypoint.TYPE_ROUND)
            waypoint_gps.points.append(coords)
            
            waypoint_pub.publish(waypoint_gps)

        elif cur_slider is () and (sim_mode is SimMode.DEFAULT or sim_mode is SimMode.CONTROLLER) and state.challenge is BoatState.CHA_STATION and button == GLUT_LEFT_BUTTON:
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
            square_pub.publish(gps_bounding_box)
            
            waypoint_pub.publish(waypoint_gps)

        elif cur_slider is () and (sim_mode is SimMode.DEFAULT or sim_mode is SimMode.CONTROLLER) and state.challenge is BoatState.CHA_SEARCH and button == GLUT_LEFT_BUTTON:
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
            
            search_area_pub.publish(gps_search_area)

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
        
        # set self.camera_velocity when mouse is near edge of screen or edge of info display panel
        if (x < 55 and x > 5 and not self.display_data.show_details) or (x < 180+50 and x > 180 and self.display_data.show_details):
            self.camera_velocity.x = -self.camera_move_speed / self.camera.scale
        elif x > self.display_data.win_width-120-50:
            self.camera_velocity.x = self.camera_move_speed / self.camera.scale
        else:
            self.all_data.ros_data["camera_velocity"].x = 0
        # The > 5 makes sure that when we scroll off the window, the self.camera stops moving
        if y < 55 and y > 5:
            self.camera_velocity.y = self.camera_move_speed / self.camera.scale
        elif y > self.display_data.win_height-55 and y < self.display_data.win_height-5:
            self.camera_velocity.y = -self.camera_move_speed / self.camera.scale
        else:
            self.camera_velocity.y = 0	

    # Handler for mouse dragging
    def motion_handler(x,y):
        if cur_slider is not ():
            cur_slider.handle_mouse(x,y)


    # Handler for all key presses that cannot be represented by an ASCII code
    def keyboard_handler(key, mousex, mousey):
        global joy
        if key == GLUT_KEY_LEFT and should_sim_joy:
            joy.right_stick_x -= 20
            if joy.right_stick_x < 0:
                joy.right_stick_x = 0
            joy_pub.publish(joy)
        elif key == GLUT_KEY_RIGHT and should_sim_joy:
            joy.right_stick_x += 20
            if joy.right_stick_x > Joy.JOY_RANGE:
                joy.right_stick_x = Joy.JOY_RANGE
            joy_pub.publish(joy)
        elif key == GLUT_KEY_UP and should_sim_joy:
            joy.left_stick_y += 50
            if joy.left_stick_y > Joy.JOY_RANGE:
                joy.left_stick_y = Joy.JOY_RANGE
            joy_pub.publish(joy)
        elif key == GLUT_KEY_DOWN and should_sim_joy:
            joy.left_stick_y -= 50
            if joy.left_stick_y < 0:
                joy.left_stick_y = 0
            joy_pub.publish(joy)


    # Handler for all key presses that can be represented by an ASCII code
    def ASCII_handler(key, mousex, mousey):
        # Handle cheat codes
        cur_input += key;
        valid = False
        for code in boat_imgs:
            if code == cur_input:
                cur_boat_img = boat_imgs[code]
                cur_rudder_img = rudder_imgs[code]
                cur_sail_img = sail_imgs[code]
                valid = False
                break
            if code.startswith(cur_input):
                valid = True
        if not valid:
            cur_input = ""
        
        if key is chr(27):
            sim_is_running = False
        elif key is 'p':
            pause_sim()
        elif key is 'm':
            if sim_mode is SimMode.DEFAULT:
                sim_mode = SimMode.REPLAY
                reset_origin_on_next_gps = True
            elif sim_mode is SimMode.REPLAY:
                sim_mode = SimMode.CONTROLLER
                reset_origin_on_next_gps = True
            else:
                sim_mode = SimMode.DEFAULT
                reset_origin_on_next_gps = False
            print 'Changed sim mode, is now', sim_mode
        elif key is 'i':
            show_details = not show_details
        elif key is 'c':
            path = PointArray()
            display_path = not display_path
        elif key is 'y':
            follow_boat = not follow_boat
        elif key is '0':
            sound = not sound
        
        elif key is 'x' and (sim_mode is SimMode.DEFAULT or sim_mode is SimMode.CONTROLLER):
            (lps_x,lps_y) = self.camera.screen_to_lps(mousex,mousey)
            coords = Waypoint(to_gps(Point(lps_x, lps_y)), Waypoint.TYPE_ROUND)
            waypoint_gps.points.append(coords)
            waypoint_pub.publish(waypoint_gps)
        
        if sim_mode is SimMode.DEFAULT or sim_mode is SimMode.CONTROLLER:
            if key is '1' and should_sim_joy:
                joy.switch_a = Joy.SWITCH_MIDDLE
                joy_pub.publish(joy)
            elif key is '2' and should_sim_joy:
                joy.switch_a = Joy.SWITCH_UP
                joy_pub.publish(joy)
            elif key is '3' and should_sim_joy:
                joy.switch_a = Joy.SWITCH_DOWN
                joy_pub.publish(joy)
            elif key is '4' and should_sim_joy:
                joy.vr = max(joy.vr - 200, 0)
                joy_pub.publish(joy)
            elif key is '5' and should_sim_joy:
                joy.vr = min(joy.vr + 200, 1000-1)
                joy_pub.publish(joy)
            elif key is 'q':
                joy.right_stick_x = Joy.JOY_RANGE/2.0
                joy_pub.publish(joy)
            elif key is 'a':
                wind_heading += 5
                wind_heading = wind_heading % 360
            elif key is 'd':
                wind_heading -= 5
                if wind_heading < 0:
                    wind_heading += 360
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