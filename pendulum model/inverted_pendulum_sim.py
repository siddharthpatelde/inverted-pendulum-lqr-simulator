import math

import pygame

from inverted_pendulum_model import InvertedPendulumCart, clamp
from pendulum_state_space_lqr_model import InvertedPendulumLQRController


# -------------------------- Plant parameters --------------------------
CART_MASS = 1.00
PENDULUM_MASS = 0.20
PENDULUM_LENGTH = 0.75
GRAVITY = 9.81
MAX_FORCE = 18.00

TARGET_X = 0.00
INITIAL_ANGLE_DEGREES = 12.00


# ------------------------- Controller weights -------------------------
LQR_Q_X = 15.00
LQR_Q_XDOT = 1.00
LQR_Q_THETA = 200.00
LQR_Q_THETADOT = 10.00
LQR_R_U = 0.50


# -------------------------- World settings ----------------------------
WORLD_X_MIN = -2.50
WORLD_X_MAX = 2.50
WORLD_Y_MIN = -0.25
WORLD_Y_MAX = 1.25
GRID_STEP = 0.25
TARGET_STEP = 0.10
PATH_SAMPLE_TIME = 0.04

DT = 1.0 / 120.0
FPS = 60


# -------------------------- Display settings --------------------------
WIDTH, HEIGHT = 1280, 820
MARGIN_LEFT = 92
MARGIN_RIGHT = 330
MARGIN_TOP = 52
MARGIN_BOTTOM = 108
PLOT_HEIGHT = 142
PLOT_TOP_GAP = 38
LIVE_PARAMETERS_GAP = 42

WHITE = (255, 255, 255)
BLACK = (5, 8, 18)
BLUE = (0, 65, 145)
BLUE_LIGHT = (224, 236, 250)
BLUE_FAINT = (244, 249, 255)
RED = (166, 24, 30)
RED_LIGHT = (250, 224, 226)
GRAY = (213, 223, 235)
GRAY_DARK = (84, 94, 112)

BG = WHITE
PLOT_BG = WHITE
PANEL_BG = WHITE
PANEL_EDGE = BLACK
GRID_MINOR = BLUE_FAINT
GRID_MAJOR = BLUE_LIGHT
AXIS = BLACK
TEXT = BLACK
MUTED = BLUE
PANEL_TEXT = BLACK
PANEL_MUTED = BLUE
CART_BODY = BLUE
CART_TOP = (16, 91, 175)
CART_EDGE = BLACK
PENDULUM = BLACK
PENDULUM_BOB = RED
RAIL = BLACK
TARGET = RED
PATH = BLUE
FORCE_NEG = RED
FORCE_POS = BLUE
INPUT_BG = WHITE
INPUT_ACTIVE = BLUE
INPUT_ERROR = RED
BUTTON_BG = WHITE
BUTTON_HOVER = BLUE_LIGHT
BUTTON_EDGE = BLUE

EDITABLE_PARAMETERS = {
    "CART_MASS": {"label": "M", "min": 0.10, "max": 20.00, "unit": "kg"},
    "PENDULUM_MASS": {"label": "m", "min": 0.01, "max": 5.00, "unit": "kg"},
    "PENDULUM_LENGTH": {"label": "l", "min": 0.20, "max": 2.00, "unit": "m"},
    "MAX_FORCE": {"label": "Fmax", "min": 1.00, "max": 80.00, "unit": "N"},
    "INITIAL_ANGLE_DEGREES": {"label": "th0", "min": -45.00, "max": 45.00, "unit": "deg"},
    "LQR_Q_X": {"label": "Q_x", "min": 0.01, "max": 800.00, "unit": ""},
    "LQR_Q_XDOT": {"label": "Q_xd", "min": 0.00, "max": 300.00, "unit": ""},
    "LQR_Q_THETA": {"label": "Q_th", "min": 0.01, "max": 2000.00, "unit": ""},
    "LQR_Q_THETADOT": {"label": "Q_td", "min": 0.00, "max": 300.00, "unit": ""},
    "LQR_R_U": {"label": "R_u", "min": 0.01, "max": 100.00, "unit": ""},
}


class WorldView:
    def __init__(self):
        usable_w = WIDTH - MARGIN_LEFT - MARGIN_RIGHT
        usable_h = HEIGHT - MARGIN_TOP - MARGIN_BOTTOM - 86
        scale_x = usable_w / (WORLD_X_MAX - WORLD_X_MIN)
        scale_y = usable_h / (WORLD_Y_MAX - WORLD_Y_MIN)
        self.scale = min(scale_x, scale_y)
        self.width = int((WORLD_X_MAX - WORLD_X_MIN) * self.scale)
        self.height = int((WORLD_Y_MAX - WORLD_Y_MIN) * self.scale)
        self.left = MARGIN_LEFT
        self.top = MARGIN_TOP
        self.right = self.left + self.width
        self.bottom = self.top + self.height

    def to_screen(self, x, y):
        sx = self.left + (x - WORLD_X_MIN) * self.scale
        sy = self.bottom - (y - WORLD_Y_MIN) * self.scale
        return int(round(sx)), int(round(sy))

    def to_world(self, sx, sy):
        x = WORLD_X_MIN + (sx - self.left) / self.scale
        y = WORLD_Y_MIN + (self.bottom - sy) / self.scale
        return x, y


class PlotHistory:
    def __init__(self):
        self.samples = []
        self.max_samples = 520
        self.sample_clock = 0.0
        self.time = 0.0

    def clear(self):
        self.samples.clear()
        self.sample_clock = 0.0
        self.time = 0.0

    def append(self, dt, x, theta, force):
        self.time += dt
        self.sample_clock += dt
        if self.sample_clock < PATH_SAMPLE_TIME:
            return
        self.sample_clock = 0.0
        self.samples.append((self.time, x, math.degrees(theta), force))
        if len(self.samples) > self.max_samples:
            self.samples.pop(0)


def draw_text(surface, font, value, pos, color=TEXT):
    surface.blit(font.render(value, True, color), pos)


def draw_arrow(surface, color, start, end, width=3, head_len=16, head_width=11):
    pygame.draw.line(surface, color, start, end, width)
    angle = math.atan2(end[1] - start[1], end[0] - start[0])
    left = (
        end[0] - head_len * math.cos(angle) + head_width * math.sin(angle) / 2.0,
        end[1] - head_len * math.sin(angle) - head_width * math.cos(angle) / 2.0,
    )
    right = (
        end[0] - head_len * math.cos(angle) - head_width * math.sin(angle) / 2.0,
        end[1] - head_len * math.sin(angle) + head_width * math.cos(angle) / 2.0,
    )
    pygame.draw.polygon(surface, color, [end, left, right])


def format_parameter(name):
    value = globals()[name]
    unit = EDITABLE_PARAMETERS[name]["unit"]
    suffix = f" {unit}" if unit else ""
    return f"{value:.2f}{suffix}"


def format_elapsed_time(seconds):
    minutes = int(seconds // 60)
    remainder = seconds - minutes * 60
    if minutes:
        return f"{minutes:d}:{remainder:04.1f} s"
    return f"{remainder:.1f} s"


def apply_parameter_edit(name, value_text):
    try:
        value = float(value_text)
    except ValueError:
        return False, "Type a number, then press Enter"

    limits = EDITABLE_PARAMETERS[name]
    if value < limits["min"] or value > limits["max"]:
        return False, f"{limits['label']} must be {limits['min']} to {limits['max']}"

    globals()[name] = value
    return True, f"{limits['label']} set to {value:.2f}"


def make_cart():
    return InvertedPendulumCart(
        theta=math.radians(INITIAL_ANGLE_DEGREES),
        cart_mass=CART_MASS,
        pendulum_mass=PENDULUM_MASS,
        pendulum_length=PENDULUM_LENGTH,
        gravity=GRAVITY,
        max_force=MAX_FORCE,
    )


def make_controller():
    controller = InvertedPendulumLQRController(
        cart_mass=CART_MASS,
        pendulum_mass=PENDULUM_MASS,
        pendulum_length=PENDULUM_LENGTH,
        gravity=GRAVITY,
    )
    controller.set_weights(LQR_Q_X, LQR_Q_XDOT, LQR_Q_THETA, LQR_Q_THETADOT, LQR_R_U)
    controller.compute_gain(DT)
    return controller


def sync_model_parameters(cart, controller):
    cart.cart_mass = CART_MASS
    cart.pendulum_mass = PENDULUM_MASS
    cart.pendulum_length = PENDULUM_LENGTH
    cart.gravity = GRAVITY
    cart.max_force = MAX_FORCE
    controller.M = CART_MASS
    controller.m = PENDULUM_MASS
    controller.l = PENDULUM_LENGTH
    controller.g = GRAVITY
    controller.set_weights(LQR_Q_X, LQR_Q_XDOT, LQR_Q_THETA, LQR_Q_THETADOT, LQR_R_U)
    controller.compute_gain(DT)


def draw_icon_button(surface, rect, icon, font, mouse_pos):
    is_hover = rect.collidepoint(mouse_pos)
    pygame.draw.rect(surface, BUTTON_HOVER if is_hover else BUTTON_BG, rect)
    pygame.draw.rect(surface, BUTTON_EDGE, rect, 2)
    cx, cy = rect.center

    if icon == "play":
        pygame.draw.polygon(surface, PANEL_TEXT, [(cx - 6, cy - 10), (cx - 6, cy + 10), (cx + 10, cy)])
    elif icon == "pause":
        pygame.draw.rect(surface, PANEL_TEXT, (cx - 8, cy - 10, 5, 20), border_radius=1)
        pygame.draw.rect(surface, PANEL_TEXT, (cx + 3, cy - 10, 5, 20), border_radius=1)
    elif icon == "reset":
        pygame.draw.arc(surface, PANEL_TEXT, (cx - 12, cy - 12, 24, 24), math.radians(35), math.radians(315), 3)
        pygame.draw.polygon(surface, PANEL_TEXT, [(cx - 1, cy - 14), (cx + 11, cy - 14), (cx + 6, cy - 3)])
    elif icon == "target":
        pygame.draw.circle(surface, PANEL_TEXT, (cx, cy), 10, 3)
        pygame.draw.circle(surface, PANEL_TEXT, (cx, cy), 3)
        pygame.draw.line(surface, PANEL_TEXT, (cx - 14, cy), (cx - 5, cy), 2)
        pygame.draw.line(surface, PANEL_TEXT, (cx + 5, cy), (cx + 14, cy), 2)
        pygame.draw.line(surface, PANEL_TEXT, (cx, cy - 14), (cx, cy - 5), 2)
        pygame.draw.line(surface, PANEL_TEXT, (cx, cy + 5), (cx, cy + 14), 2)
    elif icon == "kick":
        pygame.draw.line(surface, PANEL_TEXT, (cx - 12, cy + 10), (cx + 8, cy - 10), 3)
        pygame.draw.polygon(surface, PANEL_TEXT, [(cx + 10, cy - 12), (cx + 13, cy + 1), (cx, cy - 2)])
    else:
        draw_text(surface, font, icon, (rect.x + 9, rect.y + 8), PANEL_TEXT)


def draw_header(surface, font, small_font):
    draw_text(surface, font, "Global Coordinate Frame", (MARGIN_LEFT, 16), TEXT)
    draw_text(surface, small_font, "LQR balances the pendulum while tracking the target cart position", (MARGIN_LEFT + 300, 21), MUTED)


def draw_grid(surface, view, small_font, axis_font):
    pygame.draw.rect(surface, PLOT_BG, (view.left, view.top, view.width, view.height))

    x = math.ceil(WORLD_X_MIN / GRID_STEP) * GRID_STEP
    while x <= WORLD_X_MAX + 1e-6:
        sx1, sy1 = view.to_screen(x, WORLD_Y_MIN)
        sx2, sy2 = view.to_screen(x, WORLD_Y_MAX)
        is_major = abs(round(x) - x) < 1e-6
        pygame.draw.line(surface, GRID_MAJOR if is_major else GRID_MINOR, (sx1, sy1), (sx2, sy2), 1)
        if is_major:
            draw_text(surface, small_font, f"{x:g}", (sx1 + 4, view.bottom + 8), MUTED)
        x += GRID_STEP

    y = math.ceil(WORLD_Y_MIN / GRID_STEP) * GRID_STEP
    while y <= WORLD_Y_MAX + 1e-6:
        sx1, sy1 = view.to_screen(WORLD_X_MIN, y)
        sx2, sy2 = view.to_screen(WORLD_X_MAX, y)
        is_major = abs(round(y) - y) < 1e-6
        pygame.draw.line(surface, GRID_MAJOR if is_major else GRID_MINOR, (sx1, sy1), (sx2, sy2), 1)
        if is_major:
            draw_text(surface, small_font, f"{y:g}", (view.left - 34, sy1 - 8), MUTED)
        y += GRID_STEP

    pygame.draw.rect(surface, BLACK, (view.left, view.top, view.width, view.height), 3)
    ox, oy = view.to_screen(0.0, 0.0)
    x_end = view.to_screen(WORLD_X_MAX - 0.15, 0.0)
    y_end = view.to_screen(0.0, WORLD_Y_MAX - 0.08)
    draw_arrow(surface, AXIS, (view.left + 14, oy), x_end, width=4, head_len=16, head_width=12)
    draw_arrow(surface, AXIS, (ox, view.bottom - 14), y_end, width=4, head_len=16, head_width=12)
    draw_text(surface, axis_font, "X", (x_end[0] - 18, x_end[1] + 8), AXIS)
    draw_text(surface, axis_font, "Y", (y_end[0] + 12, y_end[1] + 2), AXIS)
    draw_text(surface, axis_font, "O", (ox + 8, oy + 8), AXIS)


def draw_pendulum(surface, view, cart, target_x, history, small_font):
    rail_y = 0.0
    rail_start = view.to_screen(WORLD_X_MIN + 0.15, rail_y)
    rail_end = view.to_screen(WORLD_X_MAX - 0.15, rail_y)
    pygame.draw.line(surface, RAIL, rail_start, rail_end, 4)

    if len(history.samples) > 1:
        points = [(view.to_screen(sample[1], rail_y - 0.035)) for sample in history.samples]
        pygame.draw.lines(surface, PATH, False, points, 3)

    target_base = view.to_screen(target_x, rail_y)
    pygame.draw.line(surface, TARGET, (target_base[0], target_base[1] - 42), (target_base[0], target_base[1] + 42), 2)
    pygame.draw.circle(surface, TARGET, target_base, 8)
    pygame.draw.circle(surface, PLOT_BG, target_base, 3)
    draw_text(surface, small_font, "target", (target_base[0] + 10, target_base[1] - 30), TARGET)

    cart_w = 0.58
    cart_h = 0.12
    cart_left, cart_top = view.to_screen(cart.x - cart_w / 2.0, rail_y + cart_h / 2.0)
    cart_right, cart_bottom = view.to_screen(cart.x + cart_w / 2.0, rail_y - cart_h / 2.0)
    cart_rect = pygame.Rect(cart_left, cart_top, cart_right - cart_left, cart_bottom - cart_top)
    pygame.draw.rect(surface, CART_EDGE, cart_rect, border_radius=6)
    pygame.draw.rect(surface, CART_BODY, cart_rect.inflate(-6, -6), border_radius=5)
    pygame.draw.rect(surface, CART_TOP, (cart_rect.x + 10, cart_rect.y + 5, cart_rect.width - 20, 5), border_radius=2)

    wheel_y = cart_rect.bottom + 6
    for wx in (cart_rect.left + 34, cart_rect.right - 34):
        pygame.draw.circle(surface, CART_EDGE, (wx, wheel_y), 10)
        pygame.draw.circle(surface, WHITE, (wx, wheel_y), 5)

    pivot = (cart_rect.centerx, cart_rect.top + 4)
    tip = view.to_screen(
        cart.x + cart.pendulum_length * math.sin(cart.theta),
        rail_y + cart_h / 2.0 + cart.pendulum_length * math.cos(cart.theta),
    )
    pygame.draw.line(surface, PENDULUM, pivot, tip, 8)
    pygame.draw.circle(surface, CART_EDGE, pivot, 7)
    pygame.draw.circle(surface, PENDULUM_BOB, tip, 13)
    pygame.draw.circle(surface, CART_EDGE, tip, 13, 2)

    if abs(cart.u) > 0.05:
        force_len = clamp(abs(cart.u) / MAX_FORCE, 0.0, 1.0) * 90.0 + 20.0
        direction = 1 if cart.u > 0 else -1
        start = (cart_rect.centerx, cart_rect.centery)
        end = (cart_rect.centerx + direction * force_len, cart_rect.centery)
        draw_arrow(surface, FORCE_POS if cart.u > 0 else FORCE_NEG, start, end, width=4, head_len=15, head_width=11)

    draw_text(surface, small_font, f"u = {cart.u: .2f} N", (view.left + 18, view.top + 16), FORCE_POS if cart.u >= 0 else FORCE_NEG)


def draw_plot(surface, view, history, small_font):
    plot_rect = pygame.Rect(view.left, view.bottom + PLOT_TOP_GAP, view.width, PLOT_HEIGHT)
    inner = pygame.Rect(plot_rect.left + 70, plot_rect.top + 28, plot_rect.width - 140, plot_rect.height - 62)
    pygame.draw.rect(surface, PLOT_BG, plot_rect)
    pygame.draw.rect(surface, BLACK, plot_rect, 2)
    mid_y = inner.centery

    for frac in (0.0, 0.25, 0.5, 0.75, 1.0):
        y = inner.top + frac * inner.height
        pygame.draw.line(surface, GRID_MINOR, (inner.left, y), (inner.right, y), 1)
    pygame.draw.line(surface, GRID_MAJOR, (inner.left, mid_y), (inner.right, mid_y), 1)

    draw_text(surface, small_font, "x (m)", (plot_rect.left + 10, plot_rect.top + 8), BLUE)
    draw_text(surface, small_font, "theta (deg)", (plot_rect.left + 70, plot_rect.top + 8), RED)
    draw_text(surface, small_font, "time (s)", (plot_rect.centerx - 26, plot_rect.bottom - 18), MUTED)

    if len(history.samples) < 2:
        return

    start_time = history.samples[0][0]
    end_time = history.samples[-1][0]
    time_span = max(end_time - start_time, PATH_SAMPLE_TIME)
    max_x = max(max(abs(sample[1]) for sample in history.samples), 0.25)
    max_theta = max(max(abs(sample[2]) for sample in history.samples), 5.0)

    scale_lines = [
        (inner.top, max_x, max_theta),
        (mid_y, 0.0, 0.0),
        (inner.bottom, -max_x, -max_theta),
    ]
    for y, x_label, theta_label in scale_lines:
        draw_text(surface, small_font, f"{x_label: .2f}", (plot_rect.left + 10, int(y) - 8), BLUE)
        draw_text(surface, small_font, f"{theta_label: .0f}", (inner.right + 12, int(y) - 8), RED)

    tick_count = 4
    for tick in range(tick_count + 1):
        frac = tick / tick_count
        x = inner.left + frac * inner.width
        pygame.draw.line(surface, GRID_MINOR, (x, inner.top), (x, inner.bottom), 1)
        seconds_from_now = -(time_span * (1.0 - frac))
        draw_text(surface, small_font, f"{seconds_from_now: .0f}", (int(x) - 10, inner.bottom + 6), MUTED)

    series = (
        (1, max_x, BLUE),
        (2, max_theta, RED),
    )
    for sample_index, scale, color in series:
        points = []
        for sample in history.samples:
            x = inner.left + (sample[0] - start_time) / time_span * inner.width
            y = mid_y - sample[sample_index] / scale * (inner.height * 0.46)
            points.append((x, y))
        pygame.draw.lines(surface, color, False, points, 2)


def draw_panel(surface, cart, target_x, sim_running, elapsed_time, status_message, mouse_pos, font, small_font, title_font):
    panel_x = WIDTH - MARGIN_RIGHT + 28
    panel_rect = pygame.Rect(panel_x - 18, MARGIN_TOP, MARGIN_RIGHT - 46, HEIGHT - MARGIN_TOP - MARGIN_BOTTOM)
    pygame.draw.rect(surface, PANEL_BG, panel_rect)
    pygame.draw.rect(surface, PANEL_EDGE, panel_rect, 2)

    draw_text(surface, title_font, "Inverted Pendulum", (panel_x, MARGIN_TOP + 20), PANEL_TEXT)
    draw_text(surface, small_font, "LQR cart control simulation", (panel_x, MARGIN_TOP + 52), PANEL_MUTED)

    button_rects = {}
    button_specs = [
        ("play_pause", "pause" if sim_running else "play"),
        ("reset", "reset"),
        ("center_target", "target"),
        ("kick", "kick"),
    ]
    y = MARGIN_TOP + 76
    for offset, (name, icon) in enumerate(button_specs):
        rect = pygame.Rect(panel_x + offset * 43, y, 36, 34)
        button_rects[name] = rect
        draw_icon_button(surface, rect, icon, small_font, mouse_pos)

    draw_text(surface, small_font, "controller", (panel_x, y + 47), PANEL_MUTED)
    lqr_rect = pygame.Rect(panel_x + 104, y + 40, 72, 28)
    pygame.draw.rect(surface, BLUE_LIGHT, lqr_rect)
    pygame.draw.rect(surface, BUTTON_EDGE, lqr_rect, 2)
    draw_text(surface, small_font, "LQR", (lqr_rect.x + 18, lqr_rect.y + 6), PANEL_TEXT)

    lines = [
        ("Status", "Running" if sim_running else "Stopped"),
        ("Control", "LQR"),
        ("Time", format_elapsed_time(elapsed_time)),
        ("target x", f"{target_x: .2f} m"),
        ("x", f"{cart.x: .2f} m"),
        ("x_dot", f"{cart.x_dot: .2f} m/s"),
        ("theta", f"{math.degrees(cart.theta): .1f} deg"),
        ("theta_dot", f"{math.degrees(cart.theta_dot): .1f} deg/s"),
        ("u", f"{cart.u: .2f} N"),
        ("limit", f"+/-{MAX_FORCE:.1f} N"),
    ]

    y += 92
    for name, value in lines:
        draw_text(surface, small_font, name, (panel_x, y), PANEL_MUTED)
        draw_text(surface, small_font, value, (panel_x + 104, y), PANEL_TEXT)
        y += 21

    y += 18
    draw_text(surface, font, "Keyboard", (panel_x, y), PANEL_TEXT)
    y += 30
    for line in ["Space  play/pause", "R  reset", "Left/Right  target", "K  disturbance", "Click/drag target"]:
        draw_text(surface, small_font, line, (panel_x, y), PANEL_TEXT)
        y += 23

    y += 8
    message_color = INPUT_ERROR if status_message.startswith("Invalid") else PANEL_MUTED
    draw_text(surface, small_font, status_message, (panel_x, y), message_color)
    return button_rects


def draw_live_parameters(surface, view, font, small_font, active_param, edit_text):
    param_rects = {}
    y = view.bottom + PLOT_TOP_GAP + PLOT_HEIGHT + LIVE_PARAMETERS_GAP
    draw_text(surface, font, "Live parameters", (view.left, y), PANEL_TEXT)

    y += 32
    for index, (name, info) in enumerate(EDITABLE_PARAMETERS.items()):
        col = index % 5
        row = index // 5
        item_x = view.left + col * 170
        item_y = y + row * 36
        draw_text(surface, small_font, info["label"], (item_x, item_y + 6), PANEL_MUTED)
        rect = pygame.Rect(item_x + 54, item_y, 92, 28)
        param_rects[name] = rect
        is_active = active_param == name
        pygame.draw.rect(surface, INPUT_BG, rect)
        pygame.draw.rect(surface, INPUT_ACTIVE if is_active else GRID_MAJOR, rect, 2 if is_active else 1)
        shown_value = edit_text if is_active else format_parameter(name)
        draw_text(surface, small_font, shown_value, (rect.x + 8, rect.y + 6), TEXT)
    return param_rects


def reset_simulation(history):
    history.clear()
    return make_cart(), make_controller(), False, 0.0


def main():
    pygame.init()
    pygame.display.set_caption("Inverted Pendulum LQR Simulator")
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    clock = pygame.time.Clock()

    font = pygame.font.SysFont("Times New Roman", 20)
    small_font = pygame.font.SysFont("Times New Roman", 16)
    title_font = pygame.font.SysFont("Times New Roman", 26, bold=True)
    axis_font = pygame.font.SysFont("Times New Roman", 30, italic=True)

    view = WorldView()
    history = PlotHistory()
    cart = make_cart()
    controller = make_controller()
    target_x = TARGET_X
    sim_running = False
    elapsed_time = 0.0
    active_param = None
    edit_text = ""
    param_rects = {}
    button_rects = {}
    dragging_target = False
    status_message = "Press play to run LQR balance"
    running = True

    while running:
        frame_dt = clock.tick(FPS) / 1000.0
        mouse_pos = pygame.mouse.get_pos()

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if active_param:
                    if event.key == pygame.K_RETURN:
                        ok, message = apply_parameter_edit(active_param, edit_text)
                        status_message = message if ok else f"Invalid: {message}"
                        if ok:
                            cart, controller, sim_running, elapsed_time = reset_simulation(history)
                            active_param = None
                            edit_text = ""
                    elif event.key == pygame.K_ESCAPE:
                        active_param = None
                        edit_text = ""
                        status_message = "Edit cancelled"
                    elif event.key == pygame.K_BACKSPACE:
                        edit_text = edit_text[:-1]
                    elif event.unicode in "0123456789.-":
                        edit_text += event.unicode
                elif event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key == pygame.K_SPACE:
                    sim_running = not sim_running
                    status_message = "LQR running" if sim_running else "Simulation paused"
                elif event.key == pygame.K_r:
                    cart, controller, sim_running, elapsed_time = reset_simulation(history)
                    status_message = "Simulation reset"
                elif event.key == pygame.K_LEFT:
                    target_x = clamp(target_x - TARGET_STEP, WORLD_X_MIN + 0.25, WORLD_X_MAX - 0.25)
                    status_message = "Target moved left"
                elif event.key == pygame.K_RIGHT:
                    target_x = clamp(target_x + TARGET_STEP, WORLD_X_MIN + 0.25, WORLD_X_MAX - 0.25)
                    status_message = "Target moved right"
                elif event.key == pygame.K_k:
                    cart.theta_dot += math.radians(45.0)
                    status_message = "Disturbance applied"
            elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                clicked_param = None
                for name, rect in param_rects.items():
                    if rect.collidepoint(event.pos):
                        clicked_param = name
                        break

                clicked_button = None
                for name, rect in button_rects.items():
                    if rect.collidepoint(event.pos):
                        clicked_button = name
                        break

                if clicked_param:
                    active_param = clicked_param
                    edit_text = f"{globals()[clicked_param]:.2f}"
                    status_message = "Type a number and press Enter"
                elif clicked_button == "play_pause":
                    sim_running = not sim_running
                    status_message = "LQR running" if sim_running else "Simulation paused"
                elif clicked_button == "reset":
                    cart, controller, sim_running, elapsed_time = reset_simulation(history)
                    status_message = "Simulation reset"
                elif clicked_button == "center_target":
                    target_x = 0.0
                    status_message = "Target centered"
                elif clicked_button == "kick":
                    cart.theta_dot += math.radians(45.0)
                    status_message = "Disturbance applied"
                elif view.left <= event.pos[0] <= view.right and view.top <= event.pos[1] <= view.bottom:
                    wx, _ = view.to_world(*event.pos)
                    target_x = clamp(wx, WORLD_X_MIN + 0.25, WORLD_X_MAX - 0.25)
                    dragging_target = True
                    active_param = None
                    edit_text = ""
                    status_message = "Target selected"
            elif event.type == pygame.MOUSEBUTTONUP and event.button == 1:
                dragging_target = False
            elif event.type == pygame.MOUSEMOTION and dragging_target:
                wx, _ = view.to_world(*event.pos)
                target_x = clamp(wx, WORLD_X_MIN + 0.25, WORLD_X_MAX - 0.25)

        sync_model_parameters(cart, controller)
        if sim_running:
            remaining = frame_dt
            while remaining > 1e-9:
                step_dt = min(DT, remaining)
                control = controller.control(cart.state, x_ref=target_x)
                cart.step(control, step_dt)
                history.append(step_dt, cart.x, cart.theta, cart.u)
                elapsed_time += step_dt
                remaining -= step_dt

        screen.fill(BG)
        draw_header(screen, font, small_font)
        draw_grid(screen, view, small_font, axis_font)
        draw_pendulum(screen, view, cart, target_x, history, small_font)
        draw_plot(screen, view, history, small_font)
        button_rects = draw_panel(
            screen,
            cart,
            target_x,
            sim_running,
            elapsed_time,
            status_message,
            mouse_pos,
            font,
            small_font,
            title_font,
        )
        param_rects = draw_live_parameters(screen, view, font, small_font, active_param, edit_text)
        pygame.display.flip()

    pygame.quit()


if __name__ == "__main__":
    main()
