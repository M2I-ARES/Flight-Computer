import random

def generate_state(state):
    state.pos = [random.uniform(0,1000) for _ in range(3)]
    state.vel = [random.uniform(-50,50) for _ in range(3)]
    state.acc = [random.uniform(-10,10) for _ in range(3)]
    state.inertialQuat   = [random.uniform(-1,1) for _ in range(4)]
    state.mag = [random.uniform(-100,100) for _ in range(3)]
    state.alt = random.uniform(0,3000)