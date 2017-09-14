# Bounding Boxes

A script for image annotation with bounding boxes of traffic lights. It saves the results of manual annotation as a .json file

## Dependencies
The script was tested with Python 3.6.2 and OpenCV 3.1.0
```
OpenCV
NumPy
json
```

## How to run

To run the script you have to specify an input directory with images and, optionally, the output file name.

```
 python bbox.py -i path/to/dir/with/images/ -o annotations.json
```
## How to use

1. Run the app.
2. Enter the frame number you'd like to start with and press ENTER. 1 - the first frame, 2 - the second, and so on. It can be useful if you'd like to continue labeling process. An image will appear.
3. Draw a rectangle you'd like to lable as a traffic light from top left corner. Move your mouse with left button pressed.  Feel free to redraw it on this step.
4. Press ESC.
5. Press 1 - to mark it as red, 2 - yellow, 3 - green.
6. Press "n" - if you'd like to mark one more traffic light (and repeat steps 3-6) on the image, "r" - to label this image again (e.q. in case of incorrect labeling), "x" to exit and dump the annotation file or ENTER to go to the next image.

**Advice:** Practice with the script using a folder with small amount of images before start labeling the main dataset. If you use "x" during labeling to save and exit, please, merge your annotation .json files on your own.

## How to deal with the results

You can deal with the resulted .json file as normally:

```Python
import json

with open('annotation.json', 'r') as f:
	data = json.load(f)
for img in data:
	print("Filename:", img["filename"])
	for tl in img["traffic_lights"]:
		print("\tColor: ", tl["yolo"][0])
		print("\tBounding box: ", tl["yolo"][1:])
```
## Output format

Every "yolo" field of the output .json file containe the following:

[class_number box_x  box_y box_width box_height]

Classes: 0 - red, 1 - yellow, 2 - green

## Known issues

* It is need a lot of focus during image labeling as not all operation can be undone.
* If you use "r" - relable option and "x" on the same image, you will have to lable one more image and only after it try to save and exit with "x".
* All images should contain at least one labeled traffic light.

Feel free to upgrade the script to meet your needs!

