# Inverted Pendulum on Cart

LQR-controlled inverted pendulum on cart simulation built with Python, NumPy, and Pygame.

The repository includes:

- `pendulum model/`: source code for the simulator and LQR controller.
- `Docs/`: hardware/reference datasheets kept with the project.

## Install

```bash
cd "pendulum model"
python3 -m pip install -r requirements.txt
```

If your system blocks global installs, use a virtual environment:

```bash
cd "pendulum model"
python3 -m venv .venv
source .venv/bin/activate
python3 -m pip install -r requirements.txt
```

## Run

```bash
python3 "pendulum model/inverted_pendulum_sim.py"
```

## Controls

- Use the right-panel icon buttons to play/pause, reset, center the target, or apply a disturbance.
- Click or drag inside the coordinate frame to move the target cart position.
- Click a live parameter value to edit it.
- Type a new number and press `Enter` to apply it.
- Press `Space` to play or pause.
- Press `R` to reset.
- Press `Left` / `Right` to move the target.
- Press `K` to apply a disturbance.
- Press `Esc` to quit.
