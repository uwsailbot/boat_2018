class Slider:
    def __init__(self,x,y,w,h,callback,min_val,max_val,cur_val):
        self.x=x
        self.y=y
        self.w=w
        self.h=h
        self.callback=callback
        self.min_val=float(min_val)
        self.max_val=float(max_val)
        self.cur_val=float(cur_val)
        self.color=(0,0,0)
        callback(float(cur_val))
    
    def resize(self, x=None, y=None, w=None, h=None):
        if x is None:
            x = self.x
        if y is None:
            y = self.y
        if w is None:
            w = self.w
        if h is None:
            h = self.h
        
        self.x = x
        self.y = y
        self.w = w
        self.h = h
    
    def draw(self):
        
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        glEnable(GL_BLEND)
        glPushMatrix()
        glTranslatef(self.x, self.y, 0)
        
        (r,g,b) = self.color
        
        glColor4f(r,g,b,0.4)
        glBegin(GL_QUADS)
        glVertex2f(0,self.h)
        glVertex2f(0,0)
        glVertex2f(self.w,0)
        glVertex2f(self.w,self.h)
        glEnd()
        
        handle_x = self.w * self.cur_val / (self.max_val - self.min_val)
        glColor4f(r,g,b,0.2)
        glBegin(GL_QUADS)
        glVertex2f(handle_x-3,self.h)
        glVertex2f(handle_x-3,0)
        glVertex2f(handle_x+3,0)
        glVertex2f(handle_x+3,self.h)
        glEnd()
        
        glPopMatrix()
        
        draw_text(
            str(self.cur_val),
            self.x+0.5*self.w,
            self.y+5,
            'center',
            self.h-5,
            2.0,
            (r,g,b))
        
        glDisable(GL_BLEND)
    
    def set_color(self, r, g, b):
        self.color=(r,g,b)
        
    def change_val(self, new_val):
        val = new_val
        if val < self.min_val:
            val = self.min_val
        if val > self.max_val:
            val = self.max_val
        self.cur_val = val
        self.callback(val)
    
    def contains(self, x, y, win_height):
        local_x = x-self.x
        local_y = win_height-y-self.y
        return local_x > 0 and local_x < self.w and local_y > 0 and local_y < self.h
    
    def	handle_mouse(self, x, y):
        local_x = x-self.x
        local_x = min(local_x, self.w)
        local_x = max(local_x, 0)
        self.change_val(self.max_val * local_x / self.w)


def draw_image(texture_id, position, angle, size, tint=(1.0,1.0,1.0)):	
    glEnable(GL_TEXTURE_2D)
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
    glEnable(GL_BLEND)
    r,g,b = tint
    glColor3f(r,g,b)
    glBindTexture(GL_TEXTURE_2D,texture_id)
    
    glPushMatrix()
    glTranslatef(position[0], position[1], 0)
    glRotatef(angle, 0, 0, 1)
    
    extents_x = size[0]/2.0
    extents_y = size[1]/2.0
    
    glBegin(GL_QUADS)
    glTexCoord2d(0,1)
    glVertex2f(-extents_x,extents_y)
    glTexCoord2d(0,0)
    glVertex2f(-extents_x,-extents_y)
    glTexCoord2d(1,0)
    glVertex2f(extents_x,-extents_y)
    glTexCoord2d(1,1)
    glVertex2f(extents_x,extents_y)
    glEnd()
    
    glPopMatrix()
    glDisable(GL_TEXTURE_2D)
    glDisable(GL_BLEND)


# Render a circle centered at (x,y) with radius r
def draw_circle(r, x, y, quality=300):
    glBegin(GL_POLYGON)
    for i in range(0, quality):
        angle = 2 * math.pi * i / float(quality)
        curx = x + math.cos(angle) * r
        cury = y + math.sin(angle) * r
        glVertex2f(curx,cury)
    glEnd()


# Render the specified text with bottom left corner at (x,y)
def draw_text(text, x, y, align='left', h = 15, spacing = 2.0, tint=(0,0,0)):
    font_texture_id = loaded_font[0]
    font_map =  loaded_font[1]
    
    scale = h/32.0
    
    glEnable(GL_TEXTURE_2D)
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
    glEnable(GL_BLEND)
    r,g,b = tint
    glColor3f(r,g,b)
    glBindTexture(GL_TEXTURE_2D, font_texture_id)
    
    glPushMatrix()
    
    # add up total width
    total_width = 0
    for c in text:
        if ord(c)<32 or ord(c)>127:
            print 'invalid character to draw: ord(c)=%i' % ord(c)
            return
        elif c is ' ':
            total_width += (h/2 + spacing) * scale
        else:			
            total_width += (font_map[ord(c)-32][1] + spacing) * scale
    
    # set alignment
    if align == 'left':
        glTranslatef(x, y, 0)
    elif align == 'center':
        glTranslatef(x-(total_width/2.0), y, 0)
    elif align == 'right':
        glTranslatef(x-total_width, y, 0)
    else:
        glTranslatef(x, y, 0)
        print '%s is not a valid alignment' % align
    
    
    x_offset = 0
    for c in text:
        if c is ' ':
            x_offset += (h/2 + spacing) * scale
        else:		
            char_info = font_map[ord(c)-32]
            start_index = char_info[0]
            char_width = char_info[1] * scale
            char_height = char_info[2] * scale
            char_y_offset = char_info[3] * scale
            tex_y_start = char_info[4]
            tex_y_end = char_info[5]
            tex_x_end = char_info[6]
        
            # use x and y as bottom left corner
            # also flip these textures, since by default everything is upside down
            glBegin(GL_QUADS)
            glTexCoord2d(0, tex_y_start)
            glVertex2f(x_offset, char_y_offset + char_height)
        
            glTexCoord2d(0, tex_y_end)
            glVertex2f(x_offset, char_y_offset)
        
            glTexCoord2d(tex_x_end, tex_y_end)
            glVertex2f(x_offset + char_width, char_y_offset)
        
            glTexCoord2d(tex_x_end, tex_y_start)
            glVertex2f(x_offset + char_width, char_y_offset + char_height)
            glEnd()
        
            x_offset += char_width + (spacing * scale)
    
    glPopMatrix()
    glDisable(GL_TEXTURE_2D)
    glDisable(GL_BLEND)