# Intelligent Traffic Control

This Python code is configured to be implemented on an Intel Edison Board to capture video from two USB cameras, detect motion objects in the visual area and computes the corresponding signal change routines for a semaphore, accordingly to the programmed logic for the traffic conditions. The light changes are commanded through GPIOs and processed video is then transmitted via WiFi through a local web server also generated and controlled by the main application.

The motion detection is achieved using the OpenCV library through a successive process of background subtraction, frame to frame comparison to detect motion objects, and object discrimination based on image size.

The light changing control consists on a base time-controlled routine, that commands a periodic light change under low-to zero traffic conditions. The presence of a vehicle in any of the concurrent roads triggers a request for a light change.

The processed video is transmitted to a python-based simple HTTP web server and then accessed by a remote client connected to the same network as the Intel Edison board.

# Future improvements
There a  lot of future improvements that can be added to this project. Please go to the <a href="https://github.com/joselamego/IntelligentTrafficControl/wiki">wiki</a> section and feel free to contribute!
