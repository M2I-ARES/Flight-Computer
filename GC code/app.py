"""
app.py — Hyperion Ground Station Server
Reads from serial via E32Receiver, pushes telemetry to browser via Socket.IO.

Install deps:
    pip install flask flask-socketio pyserial

Run:
    python app.py
Then open http://localhost:5000 in a browser.
"""

import time
import threading
from flask import Flask, render_template
from flask_socketio import SocketIO
from E32receiver import E32Receiver   # your existing receiver.py

# ── Config ────────────────────────────────────────────────────────────────────
SERIAL_PORT      = '/dev/ttyACM0'  # Feather USB on Pi — change if different
BAUD_RATE        = 115200
PUSH_INTERVAL_S  = 0.1             # how often to push telemetry to browser (10 Hz)

# ── App setup ─────────────────────────────────────────────────────────────────
app = Flask(__name__, template_folder='templates')
app.config['SECRET_KEY'] = 'hyperion'
socketio = SocketIO(app, cors_allowed_origins='*', async_mode='threading')

receiver = E32Receiver(port=SERIAL_PORT, baud=BAUD_RATE, request_interval_s=0.5)

# ── Routes ────────────────────────────────────────────────────────────────────
@app.route('/')
def index():
    return render_template('index.html')

# ── GC command endpoint (optional — POST from dashboard buttons) ──────────────
@socketio.on('command')
def handle_command(data):
    """
    Accepts commands from the browser:
        { "type": "data_request" }
        { "type": "fire_drogue" }
        { "type": "fire_main" }
    """
    cmd = data.get('type')
    if cmd == 'data_request':
        receiver.request_data()
    elif cmd == 'fire_drogue':
        receiver._gc_seq = (receiver._gc_seq + 1) & 0xFF
        from receiver import build_frame
        frame = build_frame(receiver._gc_seq, 2,
                            bytes([1, 1, 0, 1, 0]))  # fire_drogue=1
        receiver._write(frame)
    elif cmd == 'fire_main':
        receiver._gc_seq = (receiver._gc_seq + 1) & 0xFF
        from receiver import build_frame
        frame = build_frame(receiver._gc_seq, 2,
                            bytes([1, 1, 0, 0, 1]))  # fire_main=1
        receiver._write(frame)

# ── Background push thread ────────────────────────────────────────────────────
def push_telemetry():
    """Continuously push the latest rocket state to all connected browsers."""
    while True:
        state = receiver.get_state()
        payload = state.to_dict()
        socketio.emit('telemetry', payload)
        time.sleep(PUSH_INTERVAL_S)

# ── Main ──────────────────────────────────────────────────────────────────────
if __name__ == '__main__':
    receiver.start()                        # start background serial thread

    push_thread = threading.Thread(target=push_telemetry, daemon=True)
    push_thread.start()

    print("Hyperion Ground Station running at http://0.0.0.0:5000")
    socketio.run(app, host='0.0.0.0', port=5000, debug=False)
