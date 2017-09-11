# Bounding Boxes

A script for image annotation with bounding boxes of traffic lights. It saves the results of manual annotation as a .json file

## Dependencies
The script was tested with Python 3.6.2
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

1. Run the app
2. Draw a rectangle you'd like to lable as a traffic light. Feel free to redraw it on this step
3. Press ESC
4. Press 1 - to mark it as red, 2 - yellow, 3 - green. If not specified (or other key was pressed), nothing will be added to the annotations. You can use it to redraw the bounding box by pressing n on the next step
5. Press n - if you'd like to mark one more traffic light (and repeat steps 2-5), or ENTER to go to the next image
6. If there are now traffic lights on the image, you can skip it with ESC pressed 3 times

## How to deal with the results

You can deal with the resulted .json file as normally:

```Python
import json

with open('annotation.json', 'r') as f:
	data = json.load(f)
for img in data:
	print("Filename:", img["filename"])
	for tl in img["traffic_lights"]:
		print("\tColor: ", tl["color"])
		print("\tBounding box: ", tl["bbox"])

```

## Known issues

* It is need a lot of focus during image labeling as not all operation can be undone.
* The script can process only the whole directory

Feel free to upgrade the script to meet your needs!

