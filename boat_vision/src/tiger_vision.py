import sys
import math
import numpy as np
import matplotlib.pyplot as plt
import cv2
import skimage.morphology as morphology

def tiger_process(filename):
	source0 = cv2.imread(filename)
	img = source0[:, :, ::-1]
	# computed from blur Radius of 3 and stackoverflow: https://stackoverflow.com/questions/21984405/relation-between-sigma-and-radius-on-the-gaussian-blur
	blurOutput = cv2.GaussianBlur(img,(5,5),1.288)
	
	hls = cv2.cvtColor(blurOutput, cv2.COLOR_RGB2HLS).astype(np.float)
	# hls thresholds
	lower = np.array([0,102,209],dtype = "uint8")
	upper = np.array([20.5, 180, 255],dtype = "uint8")
	#TODO fix this ugly logic
	hls_mask = np.where(np.logical_and(\
										np.logical_and(\
													np.logical_and(hls[:,:,0] > lower[0] , hls[:,:,0] < upper[0]),\
													np.logical_and(hls[:,:,1] > lower[1] , hls[:,:,1] < upper[1])), \
										np.logical_and(hls[:,:,2] > lower[2] , hls[:,:,2] < upper[2])), \
										1, 0 )
	
	bgr_lower = [0, 0, 235];
	bgr_upper = [131, 191, 255];
	img = img[:, :, ::-1]
	#TODO fix this ugly logic
	bgr_mask = np.where(np.logical_and(\
										np.logical_and(\
													np.logical_and(img[:,:,0] > bgr_lower[0] , img[:,:,0] < bgr_upper[0]),\
													np.logical_and(img[:,:,1] > bgr_lower[1] , img[:,:,1] < bgr_upper[1])), \
										np.logical_and(img[:,:,2] > bgr_lower[2] , img[:,:,2] < bgr_upper[2])), \
										1, 0 )
	mask = np.logical_and(bgr_mask, hls_mask)
	im2, contours, hierarchy = cv2.findContours(mask.astype(np.uint8)*255,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
	hull = cv2.convexHull(contours[0])
	#TODO this is pretty gross
	# comment/uncomment next 4 lines for debug help
	# img = img[:, :, ::-1]
	# plt.imshow(img)
	# plt.show()
	# print(hull[0])
	return hull[0]

if __name__ == "__main__":
	if len(sys.argv) == 1:
		print("Please enter a file name")
	else:
		filename = sys.argv[1]
		tiger_process(filename)

