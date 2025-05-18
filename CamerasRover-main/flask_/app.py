import os
import requests
import base64
from flask import Flask, render_template, request, jsonify
from flask_cors import CORS
from io import BytesIO
from PIL import Image
import datetime
import io
import cv2
import time



app = Flask(__name__)
CORS(app)


SAVE_FOLDER = 'saved_images'
if not os.path.exists(SAVE_FOLDER):
    os.makedirs(SAVE_FOLDER)


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


@app.route('/save-frame', methods=['POST'])
def save_frame():
    try:
        data = request.get_json()
        image_data = data['image_data']

        header, encoded = image_data.split(',', maxsplit=1)
        binary_data = base64.b64decode(encoded)
        now = datetime.datetime.now()
        filename = f'saved_image_{now.strftime("%Y-%m-%d %H:%M:%S")}.jpg'
        filepath = os.path.join(SAVE_FOLDER, filename)

        with open(filepath, 'wb') as f:
            f.write(binary_data)

        return jsonify(success=True)
    except Exception as e:
        return jsonify(success=False, error=str(e))
    
def set_angle(pwm, angle):
    duty = angle / 18 + 2
    pwm.ChangeDutyCycle(duty)
    time.sleep(0.5)
    pwm.ChangeDutyCycle(0)   

    
RASPBERRY_PI_API = "http://10.42.0.16:5000"
    
@app.route('/servo_control', methods=['POST'])
def servo_control():
    data = request.get_json()
    print(data)

    try:
        # Raspberry Pi'ye servo kontrol isteği gönder
        response = requests.post(
            f"{RASPBERRY_PI_API}/control_servo",
            json=data,
            timeout=3
        )
        return jsonify(response.json())
    except Exception as e:
        return jsonify(success=False, error=f"Raspberry Pi'ye bağlanılamadı: {str(e)}")

    
if __name__ == '__main__':
    app.run(host='0.0.0.0', port=8000, debug=True)

