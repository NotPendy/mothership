'''
    Tests that thresholds can be read in from an xml file in proper format.

    Author: Robby Rivenbark
'''


import xml.etree.ElementTree as ET
from pathlib import Path

def get_hsv_thresholds(threshold_color) :
    threshold_list = []
    
    current_dir = Path(__file__).parent.resolve()
        
    tree = ET.parse(str(current_dir / 'test_thresholds.xml'))
    root = tree.getroot()

    color_node = root.find(threshold_color)

    for thresh in list(color_node) :
        hsv_thresh = []
        for hsv_val in list(thresh) :
            hsv_thresh.append(int(hsv_val.text))
        print(hsv_thresh)
        threshold_list.append(hsv_thresh)
    print(threshold_list)

get_hsv_thresholds("dark_red")