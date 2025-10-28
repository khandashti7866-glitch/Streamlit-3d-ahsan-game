from ursina import *
from ursina.prefabs.first_person_controller import FirstPersonController
import math

app = Ursina()

window.title = "Simple 3D Car - Ursina (Playable Demo)"
window.borderless = False
window.fullscreen = False
window.exit_button.visible = True
window.fps_counter.enabled = True

# --- tunables ---
MAX_SPEED = 40.0          # max forward speed (m/s)
ACCELERATION = 30.0       # how fast speed increases
BRAKE_DECEL = 60.0        # braking deceleration
REVERSE_SPEED = 12.0
STEER_SPEED = 80.0        # degrees per second at low speed
DRAG = 4.0                # simple drag applied to velocity
GROUND_HEIGHT = 0.2

def clamp(x, lo, hi):
    return max(lo, min(hi, x))

# --- simple flat terrain ---
ground = Entity(model='plane', scale=(200,1,200), texture='white_cube', texture_scale=(100,100), collider='box', color=color.rgb(80,150,80))
ground.y = -0.5

# add a few obstacles
obstacles = []
for i in range(20):
    x = (i%5 - 2) * 8 + (i%3)*3
    z = (i//5 - 2) * 12 + (i%4)*2
    b = Entity(model='cube', scale=(2,2,2), color=color.rgb(120,80,60), position=(x,1,z), collider='box')
    obstacles.append(b)

# simple finish line & start area visuals
start_plate = Entity(model='cube', scale=(6,0.2,6), color=color.rgb(30,30,120), position=(0,0.0,-8))
finish_plate = Entity(model='cube', scale=(6,0.2,6), color=color.rgb(120,30,30), position=(0,0.0,120))

# --- Car entity (visual + collider) ---
class SimpleCar(Entity):
    def __init__(self, **kwargs):
        super().__init__()
        # root (invisible) used for position/rotation
        self.model = None
        self.collider = 'box'
        self.scale = (1.8, 0.8, 3.2)
        # visual chassis
        self.chassis = Entity(parent=self, model='cube', scale=(1.8,0.6,3.2), position=(0,0.45,0), color=color.red)
        # simple wheels
        self.wheels = []
        wheel_positions = [(-0.7, 0.15,  1.2), (0.7, 0.15,  1.2), (-0.7, 0.15, -1.2), (0.7, 0.15, -1.2)]
        for wp in wheel_positions:
            w = Entity(parent=self, model='cylinder', scale=(0.3,0.2,0.3), rotation=(0,0,90), position=wp, color=color.black)
            self.wheels.append(w)

        # physics state
        self.velocity = Vec3(0,0,0)   # world-space velocity
        self.speed = 0.0              # scalar forward speed
        self.steer = 0.0              # -1..1 steering input
        self.max_speed = MAX_SPEED

        # starting position
        self.position = (0, 0.0, -6)
        self.rotation_y = 0

        for key, value in kwargs.items():
            setattr(self, key, value)

    def forward_vector(self):
        # get forward direction (world space)
        rad = math.radians(self.rotation_y)
        return Vec3(math.sin(rad), 0, math.cos(rad)).normalized()

    def update_physics(self, dt, throttle, brake, steer_input):
        # throttle: 0..1, brake: 0..1, steer_input: -1..1
        forward = self.forward_vector()

        # accelerate / reverse / braking
        if throttle > 0:
            self.speed += ACCELERATION * throttle * dt
        elif brake > 0:
            # strong braking
            self.speed -= BRAKE_DECEL * brake * dt
        else:
            # natural drag slow down
            if self.speed > 0:
                self.speed -= DRAG * dt
            elif self.speed < 0:
                self.speed += DRAG * dt

        # clamp forward/reverse speed
        if self.speed > self.max_speed:
            self.speed = self.max_speed
        if self.speed < -REVERSE_SPEED:
            self.speed = -REVERSE_SPEED
        # apply steering: steer more effective at low speed, less at high speed
        steer_effectiveness = clamp(1.0 - abs(self.speed) / (self.max_speed*1.2), 0.2, 1.0)
        turn_amount = steer_input * STEER_SPEED * steer_effectiveness * dt
        # update rotation
        self.rotation_y += turn_amount * (1 if self.speed >= 0 else -1)

        # update velocity and position
        self.velocity = forward * self.speed
        # apply vertical small gravity snap to ground (keep car on ground)
        self.y = GROUND_HEIGHT

        # move
        self.position += self.velocity * dt

        # simple collisions with obstacles: if overlapping, push back and reduce speed
        hit = None
        for o in obstacles:
            if self.intersects(o).hit:
                hit = o
                break
        if hit:
            # simple response: back up along reverse of velocity and reduce speed
            push_back = -self.velocity.normalized() if self.velocity.length() > 0 else Vec3(0,0,-1)
            self.position += push_back * 0.5
            self.speed *= -0.3  # bounce back and lose most speed
            camera.shake(0.03)

        # wheel visuals rotate based on speed
        for w in self.wheels:
            # rotate wheels around their local x based on linear speed
            circ = 2 * math.pi * 0.3  # approximate circumference
            if circ != 0:
                rot = (self.speed * dt / circ) * 360
                w.rotation_x += rot

# instantiate
car = SimpleCar()

# camera follow logic
camera_parent = Entity()
camera.parent = camera_parent
camera.position = (0, 6, -12)
camera.rotation_x = 12

def update_camera():
    # make the camera_parent follow the car with slight lag
    target_pos = car.position + Vec3(0, 2.0, 0)
    camera_parent.position = lerp(camera_parent.position, target_pos, time.dt * 6)
    # align camera_parent rotation to car's rotation (yaw only)
    camera_parent.rotation_y = lerp(camera_parent.rotation_y, car.rotation_y, time.dt * 6)
    # camera local offset remains (0,6,-12)

# hud
speed_text = Text(text='Speed: 0', position=(-0.85, 0.45), scale=1.2)
instr = Text(text='W/S accelerate/brake  A/D steer  R reset  C toggle camera', position=(-0.8,-0.47), scale=0.75, color=color.gray)

camera_mode = 0
camera_modes = ['chase', 'top']

def toggle_camera():
    global camera_mode
    camera_mode = (camera_mode + 1) % len(camera_modes)
    if camera_modes[camera_mode] == 'chase':
        camera.position = (0,6,-12)
        camera.rotation_x = 12
        camera.fov = 70
    elif camera_modes[camera_mode] == 'top':
        camera.position = (0,30,0)
        camera.rotation_x = 90
        camera.fov = 60

toggle_camera()

# input handling
held = held_keys

def update():
    dt = time.dt
    throttle = 0.0
    brake = 0.0
    steer_input = 0.0

    if held['w'] or held['up arrow']:
        throttle = 1.0
    if held['s'] or held['down arrow']:
        brake = 1.0
    if held['a'] or held['left arrow']:
        steer_input = -1.0
    if held['d'] or held['right arrow']:
        steer_input = 1.0

    # reset
    if held['r']:
        car.position = (0,GROUND_HEIGHT,-6)
        car.rotation_y = 0
        car.speed = 0
        car.velocity = Vec3(0,0,0)

    # camera toggle (press once)
    if held['c']:
        toggle_camera()
        held['c'] = False

    car.update_physics(dt, throttle, brake, steer_input)
    update_camera()
    speed_text.text = f"Speed: {int(car.speed * 3.6)} km/h"  # convert m/s to km/h approx

# simple sky and light
Sky()
DirectionalLight(rotation=(45,-30,0))
AmbientLight(color=color.rgba(100,100,100,0.5))

# title
title = Text("Simple 3D Car (Ursina) — playable demo", origin=(0,0), y=0.45, scale=1.2)

app.run()
