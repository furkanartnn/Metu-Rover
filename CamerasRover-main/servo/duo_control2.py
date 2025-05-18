from flask import Flask, request, jsonify
import time
import RPi.GPIO as GPIO
from flask_cors import CORS


app = Flask(__name__)
CORS(app)


servoPin1 = 32
servoPin2 = 33

GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)
GPIO.setup(servoPin1, GPIO.OUT)
GPIO.setup(servoPin2, GPIO.OUT)

pwm1 = GPIO.PWM(servoPin1, 50)
pwm2 = GPIO.PWM(servoPin2, 50)
pwm1.start(0)
pwm2.start(0)

def set_angle(pwm, angle):
    duty = angle / 18 + 2
    pwm.ChangeDutyCycle(duty)
    time.sleep(0.5)
    pwm.ChangeDutyCycle(0)



@app.route('/control_servo', methods=['POST'])
def servo_control():
    data = request.get_json()
    motor = data.get('servo_id')
    angle = int(data.get('angle'))


    try:
        if motor == '1':
            for i in range(0, angle, 1):
                set_angle(pwm1, i)
                time.sleep(0.05)
        elif motor == '2':
            for j in range(0, angle, 1):
                set_angle(pwm2, j)
                time.sleep(0.05)
        else:
            return jsonify(success=False, error="Invalid motor number.")
        
        return jsonify(success=True, motor=motor, angle=angle)
    except Exception as e:
        return jsonify(success=False, error=str(e))
    
if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)
    
    
