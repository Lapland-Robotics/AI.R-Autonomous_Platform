from flask import Flask, render_template, Response, request, jsonify, redirect, url_for
from aiortc import RTCPeerConnection, RTCSessionDescription
import cv2
import json
import uuid
import asyncio
import logging
import time
import numpy as np
import pyautogui

def capture_screen():
    screen = pyautogui.screenshot()
    frame = np.array(screen)
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    #frame = cv2.resize(frame, (1280, 720))  # Resize to HD resolution
    return frame
start_time = time.time()
camera = cv2.VideoCapture(0)
camera.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
end_time = time.time()
print("Camera loading time: ", end_time - start_time)

app = Flask(__name__, static_url_path='/static')

@app.route('/')
def index():
    return render_template('index.html')

def gen():
    while True:
        success, frame = camera.read()
        if not success:
            break
        ret, frame = cv2.imencode('.jpg', frame)
        frame = frame.tobytes()
        yield (b'--frame\r\n'
            b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

@app.route('/video_feed')
def video_feed():
    return Response(gen(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


if __name__ == "__main__":
    app.run(debug=True, host='0.0.0.0')
