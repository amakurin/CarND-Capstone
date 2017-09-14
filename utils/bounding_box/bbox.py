import os
import cv2
import json
import argparse
import numpy as np

def parse_arguments():
	parser = argparse.ArgumentParser(description = 'Bounding box image labling utility')
	parser.add_argument(
		'-i', '--input', required = True,
		help='Path to the directory with input images')
	parser.add_argument(
		'-o', '--output', default = 'annot.json',
		help='Name of the .json output file')
	args = parser.parse_args()
	return args

def lable_img(img, data):
	stop = False
	r = cv2.selectROI("Image", img, fromCenter=False)
	r = [ int(x) for x in r ]
	print("press 1 - for red, 2 - for yellow, 3 - for green")
	a=[]
	key = cv2.waitKey(0)
	if key == 49: # 1 is pressed
		a = [0]
	elif key == 50: # 2
		a = [1]
	elif key == 51: # 3
		a = [2]
	a.extend(r)
	data.append({"yolo": a})
	# User input if we need more bounding boxes
	print("Press \"n\" for new bounding box or \"r\" to relable or \"x\" to exit or ENTER to go to the next image")
	key = cv2.waitKey(0)
	if key == 110: # n - Add new boundin box on the image
		lable_img(img, data)
	if key == 114: # r - Relabele the image again
		print("Redraw labels for the frame")
		data = []
		lable_img(img, data)
	if key == 120: # x - stop the process as is and dump the json
		stop = True
	return data, stop

def main():
	args = parse_arguments()
	n_frames = len(os.listdir(args.input)) #number of frames to lable
	print("The are", n_frames, "images")
	n_start = int(input("Which frame use to start? 1 - lable all frames, start from begining"))
	if n_start > n_frames or n_start <= 0:
		n_start = 1
	n_frames = n_frames - n_start + 1
	with open(args.output, mode='w') as f: # Init .json file
		json.dump([], f)
	count = 1 # Frame counter
	with open(args.output, mode='w') as feedsjson:
		feeds = []
		for fname in os.listdir(args.input)[(n_start-1):]:
			print("File:", fname, "Frame:", count, "/", n_frames)
			img = cv2.imread(os.path.join(args.input, fname))
			tl_data = []
			tl_data, stop = lable_img(img, tl_data)
			feeds.append({"filename": fname, "traffic_lights": tl_data})
			if stop:
				print("Exit! Last frame:", fname, "n =", count)
				break
			count += 1
		json.dump(feeds, feedsjson, indent = 4)

if __name__ == '__main__' :
	main()
