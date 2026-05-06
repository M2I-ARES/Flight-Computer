from flask import Flask, render_template
from flask_socketio import SocketIO
import time
import threading

from rocket_state import RocketState
from simulator import generate_state


app = Flask(__name__)
socketio = SocketIO(app)

state = RocketState()

@app.route("/")
def index():
    return render_template("index.html")

def telemetry_loop():
    while True:
        generate_state(state)

        socketio.emit("telemetry", {
            "pos": state.pos,
            "vel": state.vel,
            "acc": state.acc,
            "quat": state.q,
            "mag": state.mag,
            "alt": state.alt
        })

        time.sleep(1)   # 1 Hz update (change later)

threading.Thread(target=telemetry_loop).start()

if __name__ == "__main__":
    socketio.run(app, host="0.0.0.0", port=5000)
