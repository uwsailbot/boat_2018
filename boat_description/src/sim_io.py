#!/usr/bin/env python
import freetype
import numpy
from OpenGL.GL import *
from OpenGL.GLUT import *
from PIL import Image

def rel_to_abs_filepath(filepath):
	abs_filepath = os.path.dirname(os.path.realpath(__file__))
	while filepath.startswith('../'):
		filepath_arr = filepath.split('/')
		filepath_arr.pop(0)
		filepath = ''.join(['/'+str(s) for s in filepath_arr])[1:]
		abs_filepath_arr = abs_filepath.split('/')
		abs_filepath_arr = abs_filepath_arr[:-1]
		abs_filepath =''.join(['/'+str(s) for s in abs_filepath_arr])[1:]
	return abs_filepath + '/' + filepath


def load_font(filepath, detail):
	# load font with freetype
	face = freetype.Face(rel_to_abs_filepath(filepath))
	face.set_char_size(detail)
	
	# we're going to load every character into one texture, placed in order vertically 
	
	# this is used to store data about location and size of each char in the texture
	font_map = []
	
	# get bitmaps, including width, height, and top bearing for each char
	# finds total width and height
	bitmaps = []
	max_width = 0
	total_height = 0
	for i in range(32,128):
		face.load_char(chr(i))
		bitmap = face.glyph.bitmap
		if bitmap.width>max_width:
			max_width = bitmap.width
		total_height += bitmap.rows
		# can't just append the bitmap object
		bitmaps.append((bitmap.buffer, bitmap.width, bitmap.rows, face.glyph.bitmap_top))
	
	# build a buffer for the texture containing all chars
	pixels = []	
	index = 0
	# for each char
	for i in range(32,128):
		bitmap_buffer = bitmaps[i-32][0]
		bitmap_width = bitmaps[i-32][1]
		bitmap_height = bitmaps[i-32][2]
		bitmap_top = bitmaps[i-32][3]
		
		#add char info to font_map
		font_map.append((
			index,
			bitmap_width,
			bitmap_height,
			bitmap_top - bitmap_height, # y offset
			index/max_width/1.0/total_height, # texture y start
			(index/max_width+bitmap_height)/1.0/total_height, # texture y end
			bitmap_width/1.0/max_width # texture x end
			))
		
		# add buffer to total buffer 
		buffer_index = 0
		
		# for each row
		for j in range(0,bitmap_height):
			# for each column
			for k in range(0, max_width):
				if k < bitmap_width:
					# copy
					pixels.append(bitmap_buffer[buffer_index])
					buffer_index+=1
				else:
					# pading until end of row
					pixels.append(0)
				index+=1
		
		# padding between each character, to avoid bleeding into each other when interpolating
		for k in range(0, max_width):
			pixels.append(0)
			index+=1
	
	# setup the texture
	texture_id = glGenTextures(1)
	glBindTexture(GL_TEXTURE_2D,texture_id)
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_MIRRORED_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_MIRRORED_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	
	# convert buffer to float format
	pixels = numpy.array(pixels, dtype='float32')
	for i in range(0, pixels.size):
		pixels[i]/=256.0
	
	# make the texture
	glTexImage2D(
		GL_TEXTURE_2D,
		0,
		GL_ALPHA,
		max_width,
		total_height,
		0,
		GL_ALPHA,
		GL_FLOAT,
		pixels);
	
	return (texture_id, font_map)

def load_image(filepath, resolution):
	# loads and returns an image
	abs_filepath = rel_to_abs_filepath(filepath)
	im = Image.open(abs_filepath)
	im = im.transpose(Image.FLIP_TOP_BOTTOM)
	im = im.resize(resolution, Image.NEAREST)
	
	texture_id = glGenTextures(1)
	glBindTexture(GL_TEXTURE_2D,texture_id)
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	pixels = numpy.array(im).flatten()
	pixels = pixels.astype(numpy.float32)
	pixels = pixels / 256.0
	
	img_mode = GL_RGB
	if im.mode == 'RGBA':
		img_mode = GL_RGBA	
	
	glTexImage2D(GL_TEXTURE_2D, 0, img_mode, im.width, im.height, 0, img_mode, GL_FLOAT, pixels);
	
	return texture_id
