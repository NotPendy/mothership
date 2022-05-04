## About Vision Testing

*Some of the tests in this direcory will give you dumb python import errors or module not found errors if you run them from this directory, so run them from the `/Mothership` directory with a command akin to `python3 test/sometest.py`*

`range_detector.py` is a very useful piece of code from imutils for determining hsv thresholds. Good place to start to make sure OpenCV is working. Run with -f HSV to determine HSV thresholds. These tend to work better than RGB thresholds and are what the frame processor is currently set to use. run with -w to use webcam output or -i for an image.

`test_horizontal_centering.py` Great for testing how your thresholds work with `frame_processor`. Upon running this file you should see a window showing your webacm's output, a window showing a black and white mask, and your terminal should output a bunch of numbers (or "seeking" if your thresholds are bad). These numbers are the commanded velocities the drone would be given to center horizontally on whatever target you're showing it.

`webcam_yaw_control.py` is `test_horizontal_centering.py`'s big brother. Does pretty much the same thing, but uses `vision_controller.py` to send commands, which can be used by a drone in sitl or in the field.

Knowing this you should be able to deduce what the rest of the vision tests do. These are the most importnat ones in general.

`rotate_servo_positions.py` runs the sero through the positions it will be in for pichup, holding, and releasing the baby drone.
