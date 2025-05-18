#!/usr/bin/python3

from flask import Flask, Response, render_template, request, jsonify
from picamera2 import Picamera2
import time
import io
from PIL import Image
from flask_cors import CORS

app = Flask(__name__)
CORS(app)
picam2 = Picamera2()
picam2.configure(picam2.create_video_configuration(main={"size": (640, 480), "format":"RGB888"},controls={"ScalerCrop": (0, 0, 4608, 2592)},))
picam2.start()

@app.route('/')
def home():
    # URL of the video to be linked to
    #video1_url = "https://images.pexels.com/photos/1036623/pexels-photo-1036623.jpeg?auto=compress&cs=tinysrgb&w=1260&h=750&dpr=1"
    video1_url = "http://10.42.0.16:8000"
    video2_url = "http://10.42.0.122:8000"
    video3_url = "http://10.42.0.137:8000"
    video4_url = "https://10.42.0.122:8000"
    return render_template('trial4.html', video1_url=video1_url,video2_url=video2_url
    ,video3_url=video3_url,video4_url=video4_url)

def generate_frames():
    while True:
        image = picam2.capture_array()
        image = image[:, :, ::-1]


        # Encode to JPEG
        buffer = io.BytesIO()
        Image.fromarray(image).save(buffer, format="JPEG", quality=75)
        frame = buffer.getvalue()
        
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

        #time.sleep(0.05)  # Reduce CPU usage

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/change_resolution', methods=['POST'])
def save_image():
    data = request.get_json()
    resolution = data.get('resolution')
    fpsValue = data.get("fpsValue")
    try:
        picam2.stop()
        fps = int(fpsValue)
        if resolution =="1080p":
            resolution = (1920, 1080)
        elif resolution == "720p":
            resolution = (1280, 720)
        elif resolution == "480p":
            resolution = (720,480)
        print("geldi")
        picam2.configure(picam2.create_video_configuration(main={"size": resolution},raw={"size":(4608, 2592)},controls={"ScalerCrop": (0, 0, 4608, 2592),"FrameDurationLimits": (int(1000000/fps),int(1000000/fps))},))
        picam2.start()
        return jsonify(success=True)
    except Exception as e:
        return jsonify(success=False, error=str(e))

if __name__ == "__main__":
    app.run(host='10.42.0.16', port=5000)
