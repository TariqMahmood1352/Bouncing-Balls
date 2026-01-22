import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, FFMpegWriter
from IPython.display import Video

# -----------------------------
# PARAMETERS
# -----------------------------
g = 9.81
dt = 0.003
t_max = 18           # longer simulation → slower video (~50-65s)
fps = 30

radius = 0.05
n_balls = 6

e_ground = 0.8
e_wall = 0.9
e_ball = 1.0
spin_factor = 0.05

x_min, x_max = -1.0, 1.0
y_min, y_max = 0.0, 2.5

ground_angle = np.deg2rad(12)
ground_normal = np.array([-np.sin(ground_angle), np.cos(ground_angle)])

# Ground function (0 ≤ x ≤ x_max)
def ground_y(x):
    return np.tan(ground_angle) * x

# -----------------------------
# INITIAL CONDITIONS
# -----------------------------
np.random.seed(3)
pos = np.column_stack((
    np.random.uniform(-0.7, 0.7, n_balls),
    np.linspace(1.2, 2.2, n_balls)
))
vel = np.random.uniform(-1.0, 1.0, (n_balls, 2))
spin = np.random.uniform(-2*np.pi, 2*np.pi, n_balls)

positions = []

# -----------------------------
# WALL COLLISIONS
# -----------------------------
def handle_wall_collision(pos, vel):
    # Left wall
    mask = pos[:,0] <= x_min + radius
    vel[mask,0] = np.abs(vel[mask,0]) * e_wall
    pos[mask,0] = x_min + radius

    # Right wall
    mask = pos[:,0] >= x_max - radius
    vel[mask,0] = -np.abs(vel[mask,0]) * e_wall
    pos[mask,0] = x_max - radius

    # Ceiling
    mask = pos[:,1] >= y_max - radius
    vel[mask,1] = -np.abs(vel[mask,1]) * e_wall
    pos[mask,1] = y_max - radius

    return pos, vel

# -----------------------------
# SIMULATION LOOP
# -----------------------------
t = 0
while t < t_max:
    # Gravity
    vel[:,1] -= g * dt
    pos += vel * dt

    # Angled ground collision (only for 0 <= x <= x_max)
    for i in range(n_balls):
        if 0 <= pos[i,0] <= x_max:
            y_ground = ground_y(pos[i,0])
            if pos[i,1] <= y_ground + radius:
                # normal and tangential decomposition
                v_n = np.dot(vel[i], ground_normal) * ground_normal
                v_t = vel[i] - v_n
                vel[i] = v_t - e_ground * v_n

                # spin effect
                vel[i,0] += spin_factor * spin[i]

                # correct position
                pos[i,1] = y_ground + radius

    # Flat floor for x < 0
    mask_floor = (pos[:,0] < 0) & (pos[:,1] <= y_min + radius)
    vel[mask_floor,1] = np.abs(vel[mask_floor,1]) * e_ground
    pos[mask_floor,1] = y_min + radius

    # Ball-ball collisions (elastic)
    for i in range(n_balls):
        for j in range(i+1, n_balls):
            delta = pos[i] - pos[j]
            dist = np.linalg.norm(delta)
            if dist < 2*radius:
                normal = delta / dist
                rel_vel = vel[i] - vel[j]
                vn = np.dot(rel_vel, normal)
                if vn < 0:
                    impulse = -(1 + e_ball) * vn / 2
                    vel[i] += impulse * normal
                    vel[j] -= impulse * normal
                # resolve overlap
                overlap = 2*radius - dist
                pos[i] += normal * overlap / 2
                pos[j] -= normal * overlap / 2

    # Wall collisions
    pos, vel = handle_wall_collision(pos, vel)

    # Spin decay
    spin *= 0.999

    # Save positions
    positions.append(pos.copy())
    t += dt

positions = np.array(positions)

# -----------------------------
# ANIMATION
# -----------------------------
fig, ax = plt.subplots(figsize=(6,6))
ax.set_xlim(x_min-0.1, x_max+0.1)
ax.set_ylim(y_min, y_max)
ax.set_title("2D Multi-Ball Bouncing with Spin & Angled Ground (0→x_max)")
ax.set_xlabel("X")
ax.set_ylabel("Y")

# Draw angled ground (0 ≤ x ≤ x_max)
xg = np.linspace(0, x_max+5, 200)
yg = ground_y(xg)
ax.plot(xg, yg, 'k', lw=2)

# Draw flat floor for x < 0
ax.hlines(y_min, x_min-0.1, 0, colors='k', lw=2)

colors = ['red','blue','green','orange','purple','cyan']
balls = [ax.scatter(positions[0,i,0], positions[0,i,1], s=300, color=colors[i]) 
         for i in range(n_balls)]

skip = max(int(1 / (fps*dt) // 2), 1)  # slow video

def update(frame):
    idx = frame*skip
    if idx >= len(positions):
        idx = -1
    for i, ball in enumerate(balls):
        ball.set_offsets(positions[idx,i])
    return balls

ani = FuncAnimation(fig, update, frames=len(positions)//skip, interval=1000/fps)

# -----------------------------
# SAVE MP4
# -----------------------------
mp4_path = "multiball_spin_ground_0_to_xmax.mp4"
writer = FFMpegWriter(fps=fps)
ani.save(mp4_path, writer=writer)
plt.close(fig)

Video(mp4_path)
