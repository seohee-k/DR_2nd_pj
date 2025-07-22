from flask import Flask, render_template, request, redirect, url_for, session, flash, Response
from flask_socketio import SocketIO
import sqlite3
import cv2
import base64
import numpy as np
import psutil
import threading
import time
import logging
from logging.handlers import RotatingFileHandler
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import os
import socket

app = Flask(__name__)
app.secret_key = 'your_secret_key'
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='threading')
latest_frame_base64 = None

# 로깅 설정
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger('MyApp')
log_handler = RotatingFileHandler('app.log', maxBytes=1000000, backupCount=5)
log_handler.setFormatter(logging.Formatter('%(asctime)s - %(levelname)s - %(message)s'))
logger.addHandler(log_handler)

# ROS2 CompressedImage Subscriber
class ROS2ImageSubscriber(Node):
    def __init__(self):
        super().__init__('ros2_image_listener')
        self.subscription = self.create_subscription(
            CompressedImage,
            '/robot7/oakd/rgb/image_raw/compressed2',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        global latest_frame_base64
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if image is not None:
                _, buffer = cv2.imencode('.jpg', image)
                latest_frame_base64 = base64.b64encode(buffer).decode('utf-8')
                logger.info("Successfully processed ROS2 image frame")
            else:
                logger.warning("Received empty image frame")
        except Exception as e:
            logger.error(f"Error decoding frame: {e}")

# ROS2 and emit logic
def ros_spin():
    rclpy.init()
    node = ROS2ImageSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

def emit_ros_frames():
    while True:
        socketio.sleep(0.2)
        if latest_frame_base64:
            socketio.emit('video_feed1', {'image': latest_frame_base64})

# 로그 파일 모니터링
def follow_log_file(filename):
    try:
        with open(filename, "r") as file:
            file.seek(0, os.SEEK_END)
            while True:
                line = file.readline()
                if not line:
                    time.sleep(0.1)
                    continue
                yield line.strip()
    except FileNotFoundError:
        logger.error(f"Log file {filename} not found. Creating new file...")
        open(filename, 'a').close()
        return follow_log_file(filename)
    except Exception as e:
        logger.error(f"Error reading log file {filename}: {e}")
        return

def save_log_to_db(log_message):
    try:
        with sqlite3.connect('mydatabase.db', check_same_thread=False) as conn:
            cursor = conn.cursor()
            timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
            parts = log_message.split(" - ", 2)
            if len(parts) == 3:
                log_level = parts[1]
                message = parts[2]
            else:
                log_level = "UNKNOWN"
                message = log_message
            cursor.execute("INSERT INTO logs (timestamp, log_level, log_message) VALUES (?, ?, ?)",
                          (timestamp, log_level, message))
            conn.commit()
    except Exception as e:
        logger.error(f"Failed to save log to database: {e}")

def monitor_logs():
    logger.info("Starting log monitoring")
    for line in follow_log_file('app.log'):
        save_log_to_db(line)
        socketio.emit('log_update', {'log': line}, namespace='/')

# DB Setup
def init_user_db():
    with sqlite3.connect('mydatabase.db', check_same_thread=False) as conn:
        cursor = conn.cursor()
        cursor.execute('''
            CREATE TABLE IF NOT EXISTS users (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                username TEXT UNIQUE NOT NULL,
                password TEXT NOT NULL
            );
        ''')
        cursor.execute('''
            CREATE TABLE IF NOT EXISTS logs (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                timestamp TEXT NOT NULL,
                log_level TEXT,
                log_message TEXT
            );
        ''')
        cursor.execute('''
            CREATE TABLE IF NOT EXISTS detection_table (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                timestamp TEXT NOT NULL,
                detection_data TEXT
            );
        ''')
        conn.commit()

init_user_db()

# Routes
@app.route('/')
def home():
    if 'username' in session:
        return redirect(url_for('welcome'))
    return redirect(url_for('login'))

@app.route('/login', methods=['GET', 'POST'])
def login():
    if request.method == 'POST':
        username = request.form['username']
        password = request.form['password']
        with sqlite3.connect('mydatabase.db', check_same_thread=False) as conn:
            cursor = conn.cursor()
            cursor.execute("SELECT * FROM users WHERE username=? AND password=?", (username, password))
            user = cursor.fetchone()
            if user:
                session['username'] = username
                flash('Login successful!', 'success')
                logger.info(f"User {username} logged in")
                return redirect(url_for('welcome'))
            else:
                flash('Invalid credentials', 'danger')
                logger.warning(f"Failed login attempt for {username}")
    return render_template('login_center.html')

@app.route('/register', methods=['GET', 'POST'])
def register():
    if request.method == 'POST':
        username = request.form['username']
        password = request.form['password']
        with sqlite3.connect('mydatabase.db', check_same_thread=False) as conn:
            cursor = conn.cursor()
            try:
                cursor.execute("INSERT INTO users (username, password) VALUES (?, ?)", (username, password))
                conn.commit()
                flash('Registration successful!', 'success')
                logger.info(f"User {username} registered")
                return redirect(url_for('login'))
            except sqlite3.IntegrityError:
                flash('Username already exists.', 'danger')
                logger.warning(f"Failed registration: Username {username} already exists")
    return render_template('register.html')

@app.route('/logout')
def logout():
    username = session.get('username')
    session.pop('username', None)
    flash('Logged out.', 'info')
    logger.info(f"User {username} logged out")
    return redirect(url_for('login'))

@app.route('/welcome')
def welcome():
    if 'username' not in session:
        flash('Login required.', 'warning')
        logger.warning("Unauthorized access to welcome page")
        return redirect(url_for('login'))
    return render_template('welcome_center_two_cam.html', username=session['username'])

@app.route('/logs')
def logs_page():
    if 'username' not in session:
        flash('Login required.', 'warning')
        return redirect(url_for('login'))
    with sqlite3.connect('mydatabase.db', check_same_thread=False) as conn:
        cursor = conn.cursor()
        cursor.execute("SELECT * FROM logs ORDER BY timestamp DESC LIMIT 100")
        logs = cursor.fetchall()
    return render_template('logs.html', logs=logs)

@app.route('/video_feed2')
def video_feed2():
    return Response(generate_frames(3, draw_box=True), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/video_feed3')
def video_feed3():
    return Response(generate_frames(0, draw_box=False), mimetype='multipart/x-mixed-replace; boundary=frame')

def generate_frames(camera_id, draw_box=False):
    cap = cv2.VideoCapture(camera_id)
    if not cap.isOpened():
        logger.error(f"Failed to open camera {camera_id}")
        return
    coords = [(460, 0), (640, 0), (640, 120), (460, 120)]
    logger.info(f"Started video stream from camera {camera_id}")
    while True:
        success, frame = cap.read()
        if not success:
            logger.warning(f"Frame read failed from camera {camera_id}")
            break
        if draw_box:
            pts = np.array(coords, np.int32).reshape((-1, 1, 2))
            cv2.polylines(frame, [pts], isClosed=True, color=(0, 255, 0), thickness=2)
        _, buffer = cv2.imencode('.jpg', frame)
        yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')

@app.route('/system_monitor')
def system_monitor():
    cpu_percent = psutil.cpu_percent(interval=1)
    memory = psutil.virtual_memory()
    disk = psutil.disk_usage('/')
    system_info = {
        'cpu': cpu_percent,
        'memory_total': memory.total // (1024 * 1024),
        'memory_used': memory.used // (1024 * 1024),
        'memory_percent': memory.percent,
        'disk_total': disk.total // (1024 * 1024 * 1024),
        'disk_used': disk.used // (1024 * 1024 * 1024),
        'disk_percent': disk.percent
    }
    logger.info("System monitor page accessed")
    return render_template('system_monitor.html', info=system_info)

@app.route('/detection')
def detection_page():
    if 'username' not in session:
        flash('Login required.', 'warning')
        return redirect(url_for('login'))
    with sqlite3.connect('mydatabase.db', check_same_thread=False) as conn:
        cursor = conn.cursor()
        cursor.execute("SELECT * FROM detection_table;")
        data = cursor.fetchall()
    logger.info("Detection page accessed")
    return render_template('detection.html', data=data)

# Start ROS2 + App
def find_free_port(start_port):
    port = start_port
    while True:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            try:
                s.bind(('0.0.0.0', port))
                return port
            except OSError:
                port += 1

if __name__ == "__main__":
    port = find_free_port(5052)
    logger.info(f"Starting server on port {port}")
    threading.Thread(target=ros_spin, daemon=True).start()
    threading.Thread(target=monitor_logs, daemon=True).start()
    socketio.start_background_task(emit_ros_frames)
    try:
        socketio.run(app, host='0.0.0.0', port=port, debug=True, use_reloader=False)
    except Exception as e:
        logger.error(f"Server failed to start: {e}")
        raise