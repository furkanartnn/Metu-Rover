from flask import Flask, render_template, request, jsonify
from flask_cors import CORS
import requests

app = Flask(__name__)
CORS(app)  # CORS izinleri

# Raspberry Pi'nin IP'si (örnek: 10.42.0.16)
RASPBERRY_PI_API = "http://10.42.0.16:5000"

@app.route('/')
def home():
    return render_template('servo_control.html')

@app.route('/servo_control', methods=['POST'])
def servo_control():
    data = request.get_json()
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