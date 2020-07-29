#!/usr/bin/env python
import rospy
import os
import json
import threading
import cv2
from random import random
from camera import VideoCamera
from flask import Flask, render_template, redirect,Response,make_response,jsonify
from geometry_msgs.msg import Twist

import time
import sys
import math

# bridge = CvBridge()
# event = Event()

app = Flask(__name__)
x = 0
y = 0 
yaw = 0

def location(res):
    global x,y,yaw
    x = res.x
    y = res.y
    yaw  = res.theta



# ROS node, publisher, and parameter.
# The node is started in a separate thread to avoid conflicts with Flask.
# The parameter *disable_signals* must be set if node is not initialized
# in the main thread.

threading.Thread(target=lambda: rospy.init_node('kratos_life', disable_signals=True)).start()

def linear(dire):
    pub = rospy.Publisher('/cmd_vel',Twist,queue_size=10)
    rate = rospy.Rate(50)
    start = time.time()
    while True:

        move = Twist()
        if(dire==1):
            move.linear.x = 1
        else:
            move.linear.x = -1

        end = time.time()
        pub.publish(move)
        total = end - start
        print(total)
        if(total>3):
            break; 
        rate.sleep()
    move.linear.x = 0
    pub.publish(move)

def rotate(clock):
    start = time.time()
    
    pub = rospy.Publisher('/cmd_vel',Twist,queue_size=10)
    rotate = Twist()
    rate = rospy.Rate(10)
    while True:

        if(clock==1):
            rotate.angular.z = 1
        else:
            rotate.angular.z = -1
        end = time.time()
        total = end - start
        print(total)
        pub.publish(rotate)
        if(total>2):
            break; 
        rate.sleep()
    rotate.angular.z = 0
    pub.publish(rotate)




@app.route('/')
def default():
    return render_template('dashboard.html')



@app.route('/forward')
def forward():
    linear(1)
    print("forward")
    return jsonify({"message":"forward"})
    #return html.action()

@app.route('/backward')
def backward():
    linear(-1)
    print("backward")
    return jsonify({"message":"backward"})
    #return html.action()


@app.route('/left')
def left():
    rotate(1)
    print("left")
    return {"message":"left"}
    #return html.action()


@app.route('/right')
def right():
    rotate(-1)
    print("right")
    return {"message":"right"}
    #return html.action()


@app.route('/stop')
def stop():
    linear(0)
    print("stop")
    return {"message":"stop"}
    #return html.action()

@app.route('/graph')
def graph():
    print("graph")
    return {"message":"live graph"}


@app.route('/video',methods=["POST"])
def video():
    print("video")
    return {"message":"live stream"}
def gen(camera):
    while True:
        frame = camera.get_frame()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n\r\n')
@app.route('/video_feed')
def video_feed():
    return Response(gen(VideoCamera()),mimetype='multipart/x-mixed-replace; boundary=frame')






if __name__ == '__main__':
	app.run(host='0.0.0.0', debug=True)
