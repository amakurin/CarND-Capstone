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
	r = cv2.selectROI("Image", img, fromCenter=False)
	print(r)
	print("press 1 - for red, 2 - for yellow, 2 - for green")
	key = cv2.waitKey(0)
	if key == 49: # 1 is pressed
		data.append({"color": "red", "bbox" : r})
	elif key == 50: # 2
		data.append({"color": "yellow", "bbox" : r})
	elif key == 51: # 3
		data.append({"color": "green", "bbox" : r})
	# User input if we need more bounding boxes
	print("Press n for new bounding box or ENTER to go to the next image")
	key = cv2.waitKey(0)
	if key == 110: # n
		lable_img(img, data)
	return data

def main():
	args = parse_arguments()
	with open(args.output, mode='w') as f: # Init .json file
		json.dump([], f)
	with open(args.output, mode='w') as feedsjson:
		feeds = []
		for fname in os.listdir(args.input):
			tl_data = []
			img = cv2.imread(os.path.join(args.input, fname))
			tl_data = lable_img(img, tl_data)
			feeds.append({"filename": fname, "traffic_lights": tl_data})
		json.dump(feeds, feedsjson, indent = 4)

if __name__ == '__main__' :
	main()
	
