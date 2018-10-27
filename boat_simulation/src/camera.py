# UI objects and UI controls stuff
class Camera:
	
	def __init__(self,x,y,scale, win_width, win_height):
		self.x=x
		self.y=y
		self.scale=scale
		self.win_width = win_width
		self.win_height = win_height
	
	def lps_to_screen(self, lps_x, lps_y):
		scrn_x = (lps_x - self.x) * self.scale
		scrn_x += self.win_width/2.0
		scrn_y = (lps_y - self.y) * self.scale
		scrn_y += self.win_height/2.0
		return (scrn_x, scrn_y)
	
	def screen_to_lps(self, scrn_x, scrn_y):
		lps_x = scrn_x - self.win_width/2.0
		lps_x /= self.scale
		lps_x += self.x
		# Screen y axis is flipped
		lps_y = -1 * (scrn_y - self.win_height/2.0)
		lps_y /= self.scale
		lps_y += self.y
		return (lps_x, lps_y)
	
	def resize(self, win_width, win_height):
		self.win_width = win_width
		self.win_height = win_height