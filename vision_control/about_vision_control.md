## About Mothership Vision Control

By: Robby Rivenbark

This document will provide a high-level overview of what the different pieces of Mothership's image processing and precision control do. For more in-depth coverage, see the documentation in the source code.

### Frame Processor

`frame_processor.py` provides three high-level functions that may each be called to process a frame of video. 

- `process_frame_check_blob()` checks if the target is in the frame and returns a boolean indicating this.
- `center_vertically()` reads in a frame and outputs a vertical velocity which should bring the mothership into vertical alignment with the target.
- `center_horizontally_and_advance()` reads in a frame and outputs a vertical velocity which should bring the mothership into vertical alignment with the target. If the mothership is already aligned horizontally with the target, it will output a velocity that makes the mothership advance. *It may be a good idea to split this functionality into two functions in the future, with the advance function providing vertical and horizontal stabilization*

All other functions in frame processer are used internally. It's worth noting that all of the important image processing happens in the `__find_blobs__()` function.

### Vision Controller
`vision_controller.py` provides two high-level functions that may each be called to adjust the mothership's position based off of information from the frame processor.
- `translate_seek()` commands the mothership to slide side to side while stil facing forward and look for the target.
- `center_in_direction()` commands the mothership to center either horizontally or vertically on the target.

Note that the high-level funtions in `vision_controller.py` have a `show` parameter that is by default set to `False`. If this parameter is set to `True` then the output of the image processing will be displayed on-screen: note that if this parameter is set to `True` the program will not be able to run successfully on a headless raspberry pi: there needs to be some kind of video output. If you'd like to see the output of the vision processing while you're running this on a drone, you can set `show=True` and connect to the pi remotely using VNC.

Lastly note the unfinished `seek()` function. This function was nearly fully functional in sitl, but there were a few kinks with the timing for the rotations. I left it in the code because it may be useful to you in the future, but for the purposes of the current mothership implementation, nothing more robust than `translate_seek()` was needed.

### Thresholds.xml

Just an xml file with HSV thresholds for each color. You should be able to add as many thresholds as you like, at least until your framerate starts to decrease. Make sure that the thresholds you add are tight and don't introduce noise into the mask. Put the thresholds you add under the tag for the relevant color, and then copy whichever ones you want to use into the `<default>` section.

Advice for settig thresholds: When setting thresholds, focus on one value at a time. E.g set everything to it's most lenient value, narrow down hmin and hmax, record hmin and hmax, set everything to it's most lenient value, narrow down smin and smin, record smin and smin, set everything to it's most lenient value, narrow down vmin and vmax, record vmin and vmax. Then try everything together and see if it all works. The program you should use to set thresholds is `range_processor.py` in the `test` folder.

More advice for setting thresholds: Take a video of the actual drone camera facing your target, then set the thresholds using frames of your video. Much more reliable than trying to do it with live output from the payload at the field. The file `test/test_video_thresholds.py` can be run on your video file so you can see how your thresholds work.