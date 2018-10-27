from OpenGL.GLUT import *
from OpenGL.GL import *

from boat_msgs.msg import BoatState

from ui_elements import Slider

import utils

class OpenGLDrawing():

    def __init__(self, display_data, all_data, hmi):
        self.display_data = display_data
        self.all_data = all_data
        self.hmi = hmi
        self.camera = self.display_data.camera
        self.ros_data = all_data.ros_data

        self.init_2D(90/255.0,155/255.0,230/255.0)

        self.sliders = {}
        self.cur_slider = ()

    # Main display rendering callback
    def redraw(self):
        
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        
        glViewport(0, 0, self.display_data.win_width, self.display_data.win_height)
        
        # Update search_area coverage, need to call this often

        challenge = self.all_data.ros_data["state"].challenge 
        if challenge is BoatState.CHA_SEARCH:
            self.all_data.search_area.update_coverage()
        
        # Render stuff
        self.draw_grid()
        if challenge is BoatState.CHA_STATION:
            self.draw_bounding_box()
        if challenge is BoatState.CHA_SEARCH:
            self.draw_search_area() 
        if self.display_data.display_path:
            self.draw_path()
        self.draw_fov()
        self.draw_target_point()
        self.draw_waypoints()
        self.draw_waypoints_in_fov()
        self.draw_obstacles()
        self.draw_boat()
        self.draw_target_heading_arrow()
        self.draw_status()
        if self.display_data.show_details:
            self.draw_detailed_status()
        
        glutSwapBuffers()

    def init_2D(self, r,g,b):
        glClearColor(r,g,b,0.0)  
        glViewport(0, 0, self.display_data.win_width, self.display_data.win_height)
        glOrtho(0.0, self.display_data.win_width, 0.0, self.display_data.win_height, -1, 1)


    # Draw all of the waypoint as red dots
    def draw_waypoints(self):
        glPushMatrix()
        
        for gps in waypoint_gps.points:
            
            if gps.type is Waypoint.TYPE_ROUND:
                glColor3f(1,0.5,0)
            else:
                glColor3f(1,0,0)
            
            
            p = to_lps(gps)
            (x,y) = self.camera.lps_to_screen(p.x, p.y)
            draw_circle(0.5 * self.camera.scale,x,y)
        
        glPopMatrix()

    def draw_bounding_box(self):
        glPushMatrix()

        glColor3f(0,1,0)
        for p in gps_bounding_box.points:
            l = to_lps(p)
            (x,y) = self.camera.lps_to_screen(l.x, l.y)
            draw_circle(0.5 * self.camera.scale,x,y)

        if len(gps_bounding_box.points) == 4:
            glLineWidth(1.0)
            glBegin(GL_LINES)
            for i in range(0, 4):
                a = to_lps(gps_bounding_box.points[i])
                b = to_lps(gps_bounding_box.points[(i+1)%4])
                (x1, y1) = self.camera.lps_to_screen(a.x, a.y)
                (x2, y2) = self.camera.lps_to_screen(b.x, b.y)
                glVertex2f(x1, y1)
                glVertex2f(x2, y2)
            glEnd()
            
        glPopMatrix()
        
    def draw_search_area(self):
        glPushMatrix()

        if search_area.center is not None:
            # draw center point
            (center_x,center_y) = self.camera.lps_to_screen(search_area.center.x, search_area.center.y)
            glColor3f(0,1,0)
            draw_circle(0.5 * self.camera.scale,center_x,center_y)

            # draw circle around center
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
            glEnable(GL_BLEND)
            if search_area.radius > 0:
                # draw actual circle
                glColor4f(0, 1, 1, 0.2)
                draw_circle(search_area.radius*self.camera.scale, center_x, center_y, 50)
                glDisable(GL_BLEND)
            else:
                # preview the circle based on mouse pos
                (mouse_x, mouse_y) = self.camera.screen_to_lps(mouse_pos.x, mouse_pos.y)
                dx = (search_area.center.x - mouse_x)
                dy = (search_area.center.y - mouse_y)
                radius = math.sqrt(dx*dx+dy*dy)
                glColor4f(0, 1, 1, 0.1)
                draw_circle(radius*self.camera.scale, center_x, center_y, 50)
            glDisable(GL_BLEND)

            if search_area.target is not None:
                # draw target
                (target_x,target_y) = self.camera.lps_to_screen(search_area.target.x, search_area.target.y)
                glColor3f(1,1,0)
                draw_circle(0.5 * self.camera.scale,target_x,target_y)

                # draw coverage
                if search_area.sections is not None:
                    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
                    glEnable(GL_BLEND)
                    glColor4f(1, 1, 0, 0.1)

                    glPushMatrix()
                    
                    for section in search_area.sections:
                        section.draw(self.camera)
                        
                    glPopMatrix()
                    glDisable(GL_BLEND)

        glPopMatrix()

    def draw_target_point(self):
        if target_point is not () and state.major is BoatState.MAJ_AUTONOMOUS and state.minor is not BoatState.MIN_COMPLETE:
            glColor3f(1,1,1)
            lps_point = to_lps(target_point.pt)
            (x,y) = self.camera.lps_to_screen(lps_point.x, lps_point.y)
            draw_circle(0.7 * self.camera.scale, x, y)

    def draw_obstacles(self):
        glPushMatrix()
        
        glColor3f(0.2, 0.2, 0.2)
        for p in obstacle_points.points:
            (x,y) = self.camera.lps_to_screen(p.x, p.y)
            draw_circle(0.5 * self.camera.scale,x,y)
        
        glPopMatrix()


    # Draw boat's target heading as an arrow centered at (x, y) pointing in the target heading's dirction
    def draw_target_heading_arrow(self):
        if state.major is not BoatState.MAJ_AUTONOMOUS:
            return
        glPushMatrix()
        
        (boat_x, boat_y) = self.camera.lps_to_screen(self.ros_data["pos"].x, self.ros_data["pos"].y)
        
        glTranslatef(boat_x, boat_y, 0)
        glRotatef(target_heading, 0, 0, 1)
        
        tip_radius = 30
        arrow_height = 10
        arrow_base = 5
        
        glColor3f(255, 255, 255)
        glBegin(GL_POLYGON)
        glVertex2f(tip_radius, 0)
        glVertex2f(tip_radius - arrow_base, arrow_base/2)
        glVertex2f(tip_radius - arrow_base, -arrow_base/2)
        glEnd()
        
        glPopMatrix()

    def draw_fov(self):
        (boat_x, boat_y) = self.camera.lps_to_screen(self.ros_data["pos"].x, self.ros_data["pos"].y)
        
        # calculate points for drawing fov cone (facing up)
        resolution = 5
        angle_step = float(fov_angle) / resolution
        cone_points = PointArray()
        cone_points.points.append(Point(0, 0))
        for i in range(-resolution, resolution + 1):
            if i is not 0:
                angle = i * angle_step
                x = math.sin(math.radians(angle/2)) * fov_radius * self.camera.scale
                y = math.cos(math.radians(angle/2)) * fov_radius * self.camera.scale
                cone_points.points.append(Point(x,y))
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        glEnable(GL_BLEND)
        glColor4f(245/255.0, 150/255.0, 25/255.0, 0.1)
        
        glPushMatrix()
        glTranslatef(boat_x, boat_y, 0)
        glRotatef(heading-90, 0, 0, 1)

        glBegin(GL_POLYGON)
        for point in cone_points.points:
            glVertex2f(point.x,point.y)
        glEnd()

        glColor4f(245/255.0, 150/255.0, 25/255.0, 0.8)
        glBegin(GL_LINE_LOOP)
        for point in cone_points.points:
            glVertex2f(point.x,point.y)
        glEnd()
        glPopMatrix()

        glDisable(GL_BLEND)


    def draw_waypoints_in_fov(self):
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        glEnable(GL_BLEND)
        glColor4f(245/255.0, 200/255.0, 5/255.0, 0.3)

        for point in vision_points_gps.points:
            lps = to_lps(point)
            (x,y) = self.camera.lps_to_screen(lps.x, lps.y)
            draw_circle(0.8 * self.camera.scale, x, y)

        glDisable(GL_BLEND)


    # Draw the right-hand 'status' panel and all of its data
    def draw_status(self):
        glPushMatrix()
        
        # Control positions of stuff
        pos_offset = win_height*0.62
        state_offset = win_height*0.47
        boat_offset = win_height*0.25
        
        # Draw the box
        panel_width = 120
        glColor3f(1.0, 1.0, 1.0)
        glBegin(GL_QUADS)
        glVertex2f(win_width,win_height)
        glVertex2f(win_width, 0)
        glVertex2f(win_width-panel_width,0)
        glVertex2f(win_width-panel_width,win_height)
        glEnd()
        
        # Set font
        set_font(open_sans_font)
        
        # Draw the wind readout
        glColor3f(0.0, 0.0, 0.0)
        draw_text("Wind: %.1f" % wind_heading, win_width-60, win_height-20, 'center')
        draw_wind_arrow(win_width-60,win_height-65)
        
        # Draw wind speed text
        draw_text("Wind speed ", win_width-60, win_height-125, 'center')
        sliders["Wind speed"].resize(win_width-100, win_height-160)
        sliders["Wind speed"].draw()
        
        # Draw the boat pos
        glColor3f(0.0, 0.0, 0.0)
        draw_text("X: %.1f m" % self.ros_data["pos"].x, win_width-60, pos_offset, 'center')
        draw_text("Y: %.1f m" % self.ros_data["pos"].y, win_width-60, pos_offset-15, 'center')
        draw_text("Spd: %.1f m/s" % boat_speed, win_width-60, pos_offset-30, 'center')
        draw_text("Head: %.1f" % heading, win_width-60, pos_offset-45, 'center')
        
        # Draw the boat state
        major = ""
        if state.major is BoatState.MAJ_RC:
            major = "RC"
        elif state.major is BoatState.MAJ_AUTONOMOUS:
            major = "Auto"
        elif state.major is BoatState.MAJ_DISABLED:
            major = "Disabled"

        challenge = ""
        if state.challenge is BoatState.CHA_STATION:
            challenge = "Station"
        elif state.challenge is BoatState.CHA_NAV:
            challenge = "Navigation"
        elif state.challenge is BoatState.CHA_LONG:
            challenge = "Long Distance"
        elif state.challenge is BoatState.CHA_AVOID:
            challenge = "Avoidance"
        elif state.challenge is BoatState.CHA_SEARCH:
            challenge = "Search"
        
        minor = ""
        if state.minor is BoatState.MIN_COMPLETE:
            minor = "Complete"
        elif state.minor is BoatState.MIN_INITIALIZE:
            minor = "Initilize"
        elif state.minor is BoatState.MIN_PLANNING:
            minor = "Planning"
        elif state.minor is BoatState.MIN_TACKING:
            minor = "Tacking"

        draw_text("State", win_width-60, state_offset, 'center', 24)
        draw_text("M: " + major, win_width-60, state_offset-20, 'center')
        draw_text("m: " + minor, win_width-60, state_offset-35, 'center')
        draw_text("C: " + challenge, win_width-60, state_offset-50, 'center')
        
        # Draw the boat diagram
        draw_status_boat(win_width-60, boat_offset)
        
        # Draw the rudder and winch pos
        draw_text("Winch: %d" % winch_pos, win_width-60, boat_offset-35, 'center')
        draw_text("Rudder: %.1f" % rudder_pos, win_width-60, boat_offset-50, 'center')
        
        # Draw the simulation speed
        draw_text("Sim speed ", win_width-60, 50, 'center')
        sliders["Sim speed"].resize(win_width-100, 20)
        sliders["Sim speed"].draw()
        draw_text("Time: %.1f" % clock + "s", win_width-60, 5, 'center')
        
        #draw_speed_graph(75, 75, 100)
        
        glPopMatrix()

    # Draw another panel to extend the status panel with more details and readings
    def draw_detailed_status(self):
        global ane_reading
        global rudder_output
        global rudder_input
        global rudder_setpoint
        global rudder_enable
        global replay_gps_raw

        glPushMatrix()
        
        # width		
        panel_width = 180
            
        # Control position of stuff
        rudder_offset = 0.8
        gps_offset = 0.5
        
        # Draw the box
        glColor3f(1.0, 1.0, 1.0)
        glBegin(GL_QUADS)
        glVertex2f(0, win_height)
        glVertex2f(0, 0)
        glVertex2f(panel_width, 0)
        glVertex2f(panel_width, win_height)
        glEnd()
        
        # Set font
        set_font(open_sans_font)
        
        # Draw anemometer reading
        draw_text("Anemometer: %.1f" % ane_reading, panel_width/2, win_height-30, 'center')
        
        # Draw rudder pid values
        draw_text("Rudder PID:", panel_width/2, win_height*rudder_offset, 'center', 18)
        draw_text("output: %.1f" % rudder_output, panel_width/2, win_height*rudder_offset-20, 'center')
        draw_text("input: %.1f" % rudder_input, panel_width/2, win_height*rudder_offset-35, 'center')
        draw_text("setpoint: %.1f" % rudder_setpoint, panel_width/2, win_height*rudder_offset-50, 'center')
        draw_text("enable: %s" % rudder_enable, panel_width/2, win_height*rudder_offset-65, 'center')
        
        # Draw gps_raw values:
        draw_text("GPS Raw:", panel_width/2, win_height*gps_offset, 'center', 18)
        draw_text("status: %f" % replay_gps_raw.status, panel_width/2, win_height*gps_offset-20, 'center')
        draw_text("satellites_used: %.1f" % replay_gps_raw.satellites_used, panel_width/2, win_height*gps_offset-35, 'center')
        draw_text("lat: %.000000001f" % replay_gps_raw.latitude, panel_width/2, win_height*gps_offset-50, 'center')
        draw_text("long: %.000000001f" % replay_gps_raw.longitude, panel_width/2, win_height*gps_offset-65, 'center')
        draw_text("alt: %.0000001f" % replay_gps_raw.altitude, panel_width/2, win_height*gps_offset-80, 'center')
        draw_text("track: %.0000001f" % replay_gps_raw.track, panel_width/2, win_height*gps_offset-95, 'center')
        draw_text("speed: %.001f" % replay_gps_raw.speed, panel_width/2, win_height*gps_offset-110, 'center')
        draw_text("hdop: %.01f" % replay_gps_raw.hdop, panel_width/2, win_height*gps_offset-125, 'center')
        
        # Draw mode
        draw_text("Sim Mode:", panel_width/2, 40, 'center', 18)
        draw_text(str(sim_mode), panel_width/2, 20, 'center') 
        glPopMatrix()


    # Draw the arrow for the wind direction centered on (x, y)
    def draw_wind_arrow(self, x,y):
        draw_image(compass_img, (x, y), 0, (70,70))
        draw_image(compass_pointer_img, (x, y), wind_heading-90, (7,40))

    # Draw grid
    def draw_grid(self):
        screen_gridsize = self.display_data.gridsize * self.camera.scale
        win_width = self.display_data.win_width
        win_height = self.display_data.win_height

        if screen_gridsize < 1:
            screen_gridsize = 1
        x = (win_width/2.0 - self.camera.x * self.camera.scale) % screen_gridsize
        y = (win_height/2.0 - self.camera.y * self.camera.scale) % screen_gridsize

        glColor3f(30/255.0,118/255.0,110/255.0)
        
        for i in range(0, int(win_width/screen_gridsize)+1):
            glBegin(GL_QUADS)
            glVertex2f(x+i*screen_gridsize,win_height)
            glVertex2f(x+i*screen_gridsize,0)
            glVertex2f(x+i*screen_gridsize+1,0)
            glVertex2f(x+i*screen_gridsize+1,win_height)
            glEnd()

        for i in range(0, int(win_height/screen_gridsize)+1):
            glBegin(GL_QUADS)
            glVertex2f(win_width, y+i*screen_gridsize)
            glVertex2f(0, y+i*screen_gridsize)
            glVertex2f(0, y+i*screen_gridsize+1)
            glVertex2f(win_width, y+i*screen_gridsize+1)
            glEnd()

    # Draw the boat on the water
    def draw_boat(self):
        (x, y) = self.camera.lps_to_screen(self.ros_data["pos"].x, self.ros_data["pos"].y)

        #draw rudder
        glPushMatrix()
        glTranslatef(x, y, 0)
        glRotatef(heading-90, 0, 0, 1)
        draw_image(
            cur_rudder_img[0], # texture id
            (0, (-cur_boat_img[1][1]/2+cur_rudder_img[1][1]*0.1)*self.camera.scale), # local y coord of rudder, moves to end of boat 
            90-rudder_pos, # rudder pos
            (cur_rudder_img[1][0]*self.camera.scale,cur_rudder_img[1][1]*self.camera.scale)) # rudder visual size
        glPopMatrix()
        
        #draw boat
        draw_image(
            cur_boat_img[0],
            (x, y),
            heading-90,
            (cur_boat_img[1][0]*self.camera.scale, cur_boat_img[1][1]*self.camera.scale))
        
        #draw sail
        sail_angle = 90 * float(WINCH_MAX - winch_pos)/(WINCH_MAX - WINCH_MIN)
        if ane_reading <= 180:
            sail_angle = -sail_angle
        glPushMatrix()
        glTranslatef(x, y, 0)
        glRotatef(heading-90, 0, 0, 1)
        draw_image(
            cur_sail_img[0],
            (0.1*self.camera.scale, 0.5*self.camera.scale),
            sail_angle,
            (cur_sail_img[1][0]*self.camera.scale, cur_sail_img[1][1]*self.camera.scale))
        glPopMatrix()


    # Draw the rudder diagram centered on (x, y)
    def draw_status_boat(self, x, y):
        # draw boat
        scale = 60.0/cur_boat_img[1][1]
        draw_image(cur_boat_img[0], (x, y+15), 0, (cur_boat_img[1][0]*scale, cur_boat_img[1][1]*scale))
        
        # draw rudder
        rudder_scale = 26.0/cur_rudder_img[1][1]
        draw_image(
            cur_rudder_img[0],
            (x, y-10),
            90-rudder_pos,
            (cur_rudder_img[1][0]*rudder_scale, cur_rudder_img[1][1]*rudder_scale))
        
        sail_scale = 42.0/cur_sail_img[1][1]
        sail_angle = 90 * float(WINCH_MAX - winch_pos)/(WINCH_MAX - WINCH_MIN)
        draw_image(
            cur_sail_img[0],
            (x+1, y+20),
            sail_angle,
            (cur_sail_img[1][0]*sail_scale, cur_sail_img[1][1]*sail_scale))


    def draw_speed_graph(self, x, y, size):
        glPushMatrix()
        glTranslatef(x,y,0)
        
        glColor3f(1.0, 1.0, 1.0)
        glBegin(GL_LINES)
        glVertex2f(-size/2, 0)
        glVertex2f(size/2, 0)
        glVertex2f(0, -size/2)
        glVertex2f(0, size/2)
        glEnd()
        
        glColor3f(0.2, 0.2, 0.2)
        for head, speed in speed_graph.items():
            radius = speed/20 * size
            ang = math.radians(head-90)
            draw_circle(2, math.cos(ang)*radius, math.sin(ang)*radius)
        
        glPopMatrix()

    def draw_path(self):
        glPushMatrix()
        glColor3f(1.0, 1.0, 1.0)
        glBegin(GL_LINES)
        for i in range(0, len(path.points)-1):
            (x, y) = self.camera.lps_to_screen(path.points[i].x, path.points[i].y)
            (x_n, y_n) = self.camera.lps_to_screen(path.points[i+1].x, path.points[i+1].y)
            glVertex2f(x, y)
            glVertex2f(x_n ,y_n)
        glEnd()
        glPopMatrix()
        
    def load_image_resources(self):
        # Load all the images
        self.display_data.compass_img = utils.load_image('../media/compass.png', (256,256))
        self.display_data.compass_pointer_img = utils.load_image('../media/compass_pointer.png', (23,128))
        
        orig=utils.load_image('../media/niceboat.png', (64,128))
        self.display_data.boat_imgs["orig"] = orig, (2,4)
        orig_rudder = utils.load_image('../media/rudder.png', (32,64))
        self.display_data.rudder_imgs["orig"] = orig_rudder, (1.5,3)
        orig_sail = utils.load_image('../media/sail.png', (32,64))
        self.display_data.sail_imgs["orig"] = orig_sail, (2,4)
        
        pirate_id=utils.load_image('../media/pirate_boat.png', (39,56))
        self.display_data.boat_imgs["pirate"] = pirate_id, (3,4)
        # use orig rudder and sail	
        self.display_data.rudder_imgs["pirate"] = orig_rudder, (1.5,3)
        self.display_data.sail_imgs["pirate"] = orig_sail, (2,4)
        
        SPACE_X = utils.load_image('../media/falcon_heavy.png', (1040,5842))
        self.display_data.boat_imgs["mars"] = SPACE_X, (2,10)
        # use orig rudder
        self.display_data.rudder_imgs["mars"] = orig_rudder, (2,3)
        roadster = utils.load_image('../media/roadster.png', (128,256))
        self.display_data.sail_imgs["mars"] = roadster, (3,6)
        
        # Load stanard/orig boat by default
        self.display_data.cur_boat_img = self.display_data.boat_imgs["orig"]
        self.display_data.cur_rudder_img = self.display_data.rudder_imgs["orig"]
        self.display_data.cur_sail_img = self.display_data.sail_imgs["orig"]


    def load_font_resources(self):
        self.display_data.open_sans_font = utils.load_font('../media/OpenSans/OpenSans-Light.ttf', 2048)
        self.set_font(self.display_data.open_sans_font)


    def init_sliders(self):
        wind_speed_slider = Slider(self.display_data.win_width-100,self.display_data.win_height-160,80,25, self.hmi.wind_speed_slider_callback, 0, 15, 5)
        wind_speed_slider.set_color(0,0,0)
        self.sliders["Wind speed"] = wind_speed_slider

        sim_speed_slider = Slider(self.display_data.win_width-100,self.display_data.win_height-200,80,25, self.hmi.sim_speed_slider_callback, 0, 1000, 200)
        sim_speed_slider.set_color(0,0,0)
        self.sliders["Sim speed"] = sim_speed_slider

    def set_font(self, font):
	    self.display_data.loaded_font = font