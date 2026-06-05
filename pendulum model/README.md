# Inverted Pendulum LQR Simulation

Pygame simulation for an inverted pendulum on a cart controlled with an LQR state-space controller.

The project is split into:

- `inverted_pendulum_model.py`: inverted pendulum on cart dynamics, RK4 stepping, and state reset.
- `pendulum_state_space_lqr_model.py`: linearized state-space LQR design and controller gain calculation.
- `inverted_pendulum_sim.py`: Pygame simulator with live LQR/plant parameter tuning, target tracking, and state plotting.

## Install

```bash
python3 -m pip install -r requirements.txt
```

If your system blocks global installs, use a virtual environment:

```bash
python3 -m venv .venv
source .venv/bin/activate
python3 -m pip install -r requirements.txt
```

## Run

```bash
python3 inverted_pendulum_sim.py
```

## Controls

- Use the icon buttons on the right panel to play/pause, reset, center the target, or apply a disturbance.
- Click or drag inside the coordinate frame to move the target cart position.
- Click a live parameter value to edit it.
- Type the new number and press `Enter` to apply it.
- Press `Esc` while editing to cancel the edit.
- Press `Space` to play or pause.
- Press `R` to reset.
- Press `Left` / `Right` to move the target.
- Press `K` to apply a disturbance.
- Press `Esc` to quit.
