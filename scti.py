#!/usr/bin/env python

#   LightControl. Traffic light control using object detection.
#   Copyright (C) 2016  Jose Lamego <joselamego@outlook.com>
#   All rights reserved.
#   Redistribution and use in source and binary forms, with or without
#   modification, are permitted provided that the following conditions are met:
#
#   1. Redistributions of source code must retain the above copyright notice,
#   this list of conditions and the following disclaimer.
#
#   2. Redistributions in binary form must reproduce the above copyright
#   notice, this list of conditions and the following disclaimer in the
#   documentation and/or other materials provided with the distribution.
#
#   3. Neither the name of the copyright holder nor the names of its
#   contributors may be used to endorse or promote products derived from this
#   software without specific prior written permission.
#
#   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
#   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
#   ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
#   LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
#   CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
#   SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
#   INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
#   CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
#   ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#   POSSIBILITY OF SUCH DAMAGE.
#
# ************* Requirements **********************
# Host system must have Python 2.7.x installed and
# the following packages must be available:
# - os
# - threading
# - imutils
# - subprocess
# - sys
# - BaseHTTPServer
# - Image
# - StringIO
# - time
# - numpy
# - datetime
# - ntplib (for time-zone synchronization)

# ******* import the necessary packages ***********
import argparse
from datetime import datetime
import time
import cv2
import numpy as np
import threading
import logging
import mraa
import Image
import StringIO
from BaseHTTPServer import BaseHTTPRequestHandler, HTTPServer
from os import listdir
import sync_time
import set_server_ip

logging.basicConfig(
    filename='log', format='%(asctime)s %(message)s',
    datefmt='%m/%d/%Y %I:%M:%S %p', level=logging.DEBUG)

# *************** Variables ************************

# dilate kernel size
dilate_kernel = 11

# Defaults
def_Thresh = 30
def_ksize = 5
def_minArea = 500

# Semaphore settings
moving_line = 1
light_text = "GREEN"
green_color = (0, 255, 0)
red_color = (255, 0, 0)
yellow_color = (255, 255, 0)
no_color = (0, 0, 0)
color_1 = green_color
color_2 = no_color
color_3 = no_color
no_motion_text = "No Motion Detected"
motion_text = "Moving object detected"
change_requested = 0
thick_1_a = -1
thick_1_b = 1
thick_1_c = 1
thick_2_a = 1
thick_2_b = 1
thick_2_c = -1

# Camera settings
cameras = []
camera_width = 320
camera_hight = 240
camera_saturation = 0.2

frame_1 = None
frame_2 = None
first_frame_1 = None
first_frame_2 = None

# **************** light groups *********************

# For "N" number of lights, lights Tuples ('Name',gpio#)
# must be supplied in following order:
# "Red1, Yellow1, Green1,...,RedN, YellowN, GreenN"
#
lights_tuples = [('redLight1', 9), ('yellowLight1', 5), ('greenLight1', 11),
                 ('redLight2', 8), ('yellowLight2', 6), ('greenLight2', 10)]
lights = []

# ******************** point array *******************
# The following point arrays must be defined accordingly to the actual road
pts_1 = np.array([[299, 0], [0, 479], [639, 479], [329, 0]], np.int32)
pts_2 = np.array([[299, 0], [0, 479], [639, 479], [329, 0]], np.int32)


# ******* Time intervales in seconds ***************
# lap_period_sec - Interval between each light change
lap_period_sec = 10
# yellow_period_sec - Time for the yellow light to be On before Red
yellow_period_sec = 2
# lap_to_go - Remaining time before a light-change event
lap_to_go = lap_period_sec
# last_second - Time when previous cycle ran
last_second = datetime.now().second


def gpio_setup():
    """
    gpio_setup: define gpio pins and set them as output
    """
    global lights
    n = len(lights_tuples)
    # Configure lights as outputs and turn them OFF
    try:
        for l in range(0, n):
            local_light = lights_tuples[l][0]
            local_light = mraa.Gpio(lights_tuples[l][1])
            lights.append(local_light)
            local_light.dir(mraa.DIR_OUT)
            local_light.write(1)
            logging.info(
                'GPIO ' + str(lights_tuples[l][1]) + ' configured for ' +
                lights_tuples[l][0] + '.')
        lights[2].write(0)
        lights[3].write(0)
    except:
        logging.error('Cannot configure GPIO for ' + lights_tuples[l][0] + '.')


def turn_off_all_lights():
    global lights
    n = len(lights_tuples)

    try:
        for l in range(0, n):
            lights[l].write(1)
    except:
        logging.error('Cannot turn off all lights.')


def cameras_setup():
    global available_cameras
    try:
        available_cameras = 0
        devs = listdir('/dev/')
        for dev in devs:
            if 'video' in dev:
                available_cameras += 1
        logging.info(
            'Detected ' + str(available_cameras) +
            ' available cameras in system.')
    except:
        logging.error(
            'Cannot determine number of available cameras in system.')

    global cameras

    try:
        for n in range(0, available_cameras):
            thisCam = cv2.VideoCapture(n)
            cameras.append(thisCam)
            time.sleep(0.25)
            thisCam.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH, camera_width)
            thisCam.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT, camera_hight)
            thisCam.set(cv2.cv.CV_CAP_PROP_SATURATION, camera_saturation)
            logging.info('Configured camera' + str(n+1) + ' size to ' +
                         str(camera_width) + 'x' + str(camera_hight) +
                         ' and ' + str(camera_saturation) + ' saturation.')
    except:
        logging.error(
            'Cannot configure camera' + str(n+1) + ' size/saturation.')


class light_change(object):
    """light_change class
    The run()method will be started and
    it will run in the background.
    """
    def __init__(self, interval=yellow_period_sec):
        """ Constructor
        :type interval: int
        :param interval: Change interval in seconds
        """
        self.interval = interval

        global thread
        thread_light_change = threading.Thread(target=self.run, args=())
        thread_light_change.daemon = False
        thread_light_change.start()

    def run(self):
        """Method to change to yellow, then red light"""
        global moving_line
        global change_requested
        global thick_1_a, thick_1_b, thick_1_c, thick_2_a, thick_2_b, thick_2_c
        global lap_to_go
        global lights
        line = moving_line
        if line == 1:
            thick_1_a = 1
            lights[2].write(1)
            thick_1_b = -1
            lights[1].write(0)
            time.sleep(self.interval)
            thick_1_b = 1
            lights[1].write(1)
            thick_1_c = -1
            lights[0].write(0)
            thick_2_a = -1
            lights[5].write(0)
            thick_2_b = 1
            lights[4].write(1)
            thick_2_c = 1
            lights[3].write(1)
            line = 2
        else:
            thick_2_a = 1
            lights[5].write(1)
            thick_2_b = -1
            lights[4].write(0)
            time.sleep(self.interval)
            thick_2_b = 1
            lights[4].write(1)
            thick_2_c = -1
            lights[3].write(0)
            thick_1_a = -1
            lights[2].write(0)
            thick_1_b = 1
            lights[1].write(1)
            thick_1_c = 1
            lights[0].write(1)
            line = 1

        moving_line = line
        change_requested = 0
        lap_to_go = lap_period_sec


def nothing(x):
    """
    Do nothing
    """
    pass


def make_odd(number):
    """
    Return the input number if its odd,
    or return input number + 1 if its even or zero.
    """
    if number == 0:
        number += 1
    if number % 2 == 0:
        number += -1
    return number


def light_circles():
    """
    Draw filled circles to simulate semaphore light.
    """
    global frame_1, frame_2
    x_circle_center = int(camera_width/11)
    y_circle_center = int(camera_hight/9)
    circle_radius = int(camera_width*.07)
    cv2.circle(
        frame_1, (x_circle_center, (circle_radius+(4*circle_radius))),
        circle_radius, green_color, thickness=thick_1_a)
    cv2.circle(
        frame_1, (x_circle_center, (circle_radius+(2*circle_radius))),
        circle_radius, yellow_color, thickness=thick_1_b)
    cv2.circle(
        frame_1, (x_circle_center, circle_radius),
        circle_radius, red_color, thickness=thick_1_c)
    cv2.circle(
        frame_2, (x_circle_center, (circle_radius+(4*circle_radius))),
        circle_radius, green_color, thickness=thick_2_a)
    cv2.circle(
        frame_2, (x_circle_center, (circle_radius+(2*circle_radius))),
        circle_radius, yellow_color, thickness=thick_2_b)
    cv2.circle(
        frame_2, (x_circle_center, circle_radius),
        circle_radius, red_color, thickness=thick_2_c)


def road_lines():
    """
    Draw road lines to define valid detection areas.
    """
    cv2.polylines(frame_1, [pts_1], True, yellow_color)
    cv2.polylines(frame_2, [pts_2], True, yellow_color)


class motion_detection():
    """

    """


class cam_handler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path.endswith('.mjpg'):
            self.send_response(200)
            self.send_header(
                'Content-type',
                'multipart/x-mixed-replace; boundary=--jpgboundary')
            self.end_headers()

            # loop over the frames of the video
            while True:
                try:
                    global first_frame_1, first_frame_2
                    global frame_1, frame_2
                    # grab the current frame and initialize the
                    # "Moving object" detection message
                    (grabbed_1, frame_1) = cameras[0].read()
                    (grabbed_2, frame_2) = cameras[1].read()
                    text_1 = no_motion_text
                    text_2 = no_motion_text

                    # if one of the frames could not be grabbed,
                    # then we have reached the end of the video
                    if not (grabbed_1 and grabbed_2):
                        break

                    # Draw three circles to simulate a semaphore
                    light_circles()

                    # resize the frame, convert it to grayscale, and blur it
                    ksize = make_odd(def_ksize)
                    gray_1 = cv2.cvtColor(frame_1, cv2.COLOR_BGR2GRAY)
                    gray_2 = cv2.cvtColor(frame_2, cv2.COLOR_BGR2GRAY)
                    gray_1 = cv2.GaussianBlur(gray_1, (ksize, ksize), 0)
                    gray_2 = cv2.GaussianBlur(gray_2, (ksize, ksize), 0)

                    # if the first frame is None, initialize it
                    if first_frame_1 is None:
                        first_frame_1 = gray_1
                        first_frame_2 = gray_2
                        continue

                    # compute the absolute difference between the current
                    # frame and first frame
                    Thresh = def_Thresh
                    frame_delta_1 = cv2.absdiff(first_frame_1, gray_1)
                    frame_delta_2 = cv2.absdiff(first_frame_2, gray_2)
                    thresh1 = cv2.threshold(
                        frame_delta_1, Thresh, 255, cv2.THRESH_BINARY)[1]
                    thresh2 = cv2.threshold(
                        frame_delta_2, Thresh, 255, cv2.THRESH_BINARY)[1]
                    # use current frame for next iteration comparisson
                    first_frame_1 = gray_1
                    first_frame_2 = gray_2

                    # dilate the thresholded image to fill in holes,
                    # then find contours on thresholded image
                    thresh1 = cv2.dilate(
                        thresh1, np.ones((dilate_kernel, dilate_kernel)),
                        iterations=2)
                    thresh2 = cv2.dilate(
                        thresh2, np.ones((dilate_kernel, dilate_kernel)),
                        iterations=2)
                    (cnts1, _) = cv2.findContours(
                        thresh1.copy(), cv2.RETR_EXTERNAL,
                        cv2.CHAIN_APPROX_SIMPLE)
                    (cnts2, _) = cv2.findContours(
                        thresh2.copy(), cv2.RETR_EXTERNAL,
                        cv2.CHAIN_APPROX_SIMPLE)

                    # loop over the contours
                    for c1 in cnts1:

                        # if the contour is too small, ignore it
                        if cv2.contourArea(c1) < args["min_area"]:
                            continue

                        # compute the bounding box for the contour,
                        # draw it on the frame, and update the text
                        (x, y, w, h) = cv2.boundingRect(c1)
                        cv2.rectangle(
                            frame_1, (x, y), (x + w, y + h), (0, 255, 0), 2)
                        text_1 = motion_text

                    for c2 in cnts2:

                        # if the contour is too small, ignore it
                        if cv2.contourArea(c2) < args["min_area"]:
                            continue

                        # compute the bounding box for the contour,
                        # draw it on the frame, and update the text
                        (x, y, w, h) = cv2.boundingRect(c2)
                        cv2.rectangle(
                            frame_2, (x, y), (x + w, y + h), (0, 255, 0), 2)
                        text_2 = motion_text

                    # draw the motion-detection message on the frame
                    cv2.putText(frame_1, "{}".format(text_1), (60, 20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                    cv2.putText(frame_2, "{}".format(text_2), (60, 20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

                    # Draw the timesatmp on the frame
                    cv2.putText(
                        frame_1,
                        datetime.now().strftime("%A %d %B %Y %I:%M:%S%p"),
                        (10, frame_1.shape[0] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
                    cv2.putText(
                        frame_2,
                        datetime.now().strftime("%A %d %B %Y %I:%M:%S%p"),
                        (10, frame_2.shape[0] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

                    # Draw the remaining time before next light-change
                    # and triger the light-change when lap_period_sec is
                    # completed only once per second
                    global lap_to_go
                    global last_second
                    this_second = datetime.now().second
                    if not last_second == this_second:
                        global change_requested
                        if lap_to_go <= 0:
                            light_change()
                            lap_to_go = lap_period_sec
                        if moving_line == 1:
                            if change_requested == 0:
                                if text_2 == no_motion_text:
                                    if lap_to_go <= lap_period_sec:
                                        lap_to_go += 1
                                else:
                                    change_requested = 1
                        else:
                            if change_requested == 0:
                                if text_1 == no_motion_text:
                                    if lap_to_go <= lap_period_sec:
                                        lap_to_go += 1
                                else:
                                    change_requested = 1
                        lap_to_go -= 1

                    if moving_line == 1:
                        if lap_to_go < lap_period_sec:
                            cv2.putText(
                                frame_1, "{}".format(lap_to_go), (80, 65),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, yellow_color, 2)
                    else:
                        if lap_to_go < lap_period_sec:
                            cv2.putText(
                                frame_2, "{}".format(lap_to_go), (80, 65),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, yellow_color, 2)

                    width_1 = len(frame_1[0, :])
                    heigth1 = len(frame_1[:, 0])
                    total_bytes = width_1 * heigth1 * 6

                    total_array = bytearray(total_bytes)
                    byte_array = np.array(total_array)
                    merged_frame = byte_array.reshape(heigth1, (width_1*2), 3)
                    merged_frame[0:heigth1, 0:width_1] = frame_1
                    merged_frame[0:heigth1, width_1:(width_1*2)] = frame_2

                    jpg = Image.fromarray(merged_frame)
                    tmp_file = StringIO.StringIO()
                    jpg.save(tmp_file, 'JPEG')
                    self.wfile.write("--jpgboundary")
                    self.send_header('Content-type', 'image/jpeg')
                    self.send_header('Content-length', str(tmp_file.len))
                    self.end_headers()
                    jpg.save(self.wfile, 'JPEG')
                    time.sleep(0.05)

                    last_second = this_second
                except KeyboardInterrupt:
                    break
            return
        if self.path.endswith('.html'):
            self.send_response(200)
            self.send_header('Content-type', 'text/html')
            self.end_headers()
            self.wfile.write('<html><head></head><body>')
            mjpeg_source = "http://%s:8080/cam.mjpg" % serverIp
            print "I: cam.mjpeg sourced from %s" % mjpeg_source
            self.wfile.write('<img src=%s>' % mjpeg_source)
            self.wfile.write('</body></html>')
            return


def main():
    # construct the argument parser and parse the arguments
    ap = argparse.ArgumentParser(
        description="Traffic light control using object detection.")
    ap.add_argument("-a", "--min-area",
                    type=int, default=def_minArea, help="minimum area size")
    global args
    args = vars(ap.parse_args())

    # ********* System setup **************************
    gpio_setup()

    sync_time.run()
    cameras_setup()
    global serverIp
    serverIp = set_server_ip.run()

    global frame_1
    global frame_2

    # initialize the first frame in the video stream
    first_frame_1 = None
    first_frame_2 = None

    try:
        server = HTTPServer(('', 8080), cam_handler)
        print "I: Server started at %s:8080/index.html" %serverIp
        server.serve_forever()
    except KeyboardInterrupt:
        cameras[0].release()
        cameras[1].release()
        server.socket.close()
        turn_off_all_lights()

if __name__ == '__main__':
    main()
