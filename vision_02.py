# ==============================================================================
# E-task target vision optimized for CanMV/K230
# Goals:
# 1) reduce false rectangle hits by scoring A4-like black bordered targets
# 2) reduce stalls by avoiding per-frame gc/display-heavy debug work
# 3) improve control smoothness with a small lock/hold state machine
# ==============================================================================
import time, os, gc, math
from math import sqrt
from media.sensor import *
from media.display import *
from media.media import *
import cv_lite
from time import ticks_ms
from machine import UART
from machine import FPIOA
from machine import Pin

# ------------------------------------------------------------------------------
# Mode switches
# ------------------------------------------------------------------------------
DRAW_CIRCLE_MODE = True
TARGET_RADIUS_CM = 6.0
A4_SHORT_CM = 21.0
A4_LONG_CM = 29.7

# Display/debug cost is high on K230 IDE preview.
# UART still updates every processed frame; only preview is skipped.
DISPLAY_EVERY_N = 2
DRAW_DEBUG = True
DRAW_PROJECTED_CIRCLE_POINTS = False
REFINE_WITH_RED_CENTER = True

# ------------------------------------------------------------------------------
# Hardware init
# ------------------------------------------------------------------------------
fpioa = FPIOA()
fpioa.set_function(11, fpioa.UART2_TXD)
fpioa.set_function(12, fpioa.UART2_RXD)
fpioa.set_function(32, FPIOA.GPIO32)
fpioa.set_function(33, FPIOA.GPIO33)

INC_KEY = Pin(32, Pin.IN, Pin.PULL_UP)
DEC_KEY = Pin(33, Pin.IN, Pin.PULL_UP)

uart = UART(
    UART.UART2,
    baudrate=115200,
    bits=UART.EIGHTBITS,
    parity=UART.PARITY_NONE,
    stop=UART.STOPBITS_ONE,
)

DETECT_WIDTH = ALIGN_UP(800, 16)
DETECT_HEIGHT = 480
IMG_W = 480
IMG_H = 320
IMG_CX = IMG_W // 2
IMG_CY = IMG_H // 2
image_shape = [IMG_H, IMG_W]

# ------------------------------------------------------------------------------
# cv_lite rectangle detection parameters
# Keep this stage permissive. Selection is handled by scoring below.
# ------------------------------------------------------------------------------
canny_thresh1 = 25
canny_thresh2 = 95
approx_epsilon = 0.075
area_min_ratio = 0.0008
max_angle_cos = 0.98
gaussian_blur_size = 3

# ------------------------------------------------------------------------------
# Target filters and tracking
# ------------------------------------------------------------------------------
A4_RATIO = A4_LONG_CM / A4_SHORT_CM
ASPECT_MIN = 1.12
ASPECT_MAX = 1.82
MIN_SIDE_PX = 22
MAX_AREA_RATIO = 0.65
EDGE_BLACK_SUM = 235
MIN_DARK_EDGE_HITS = 2
RED_MIN_HITS = 6
RED_SCAN_STEP = 2
USE_LOOSE_RECT_FALLBACK = True
USE_NATIVE_FIND_RECTS_FALLBACK = True
USE_RED_CENTER_FALLBACK = True
RED_FALLBACK_STEP = 4
RED_GRID_SIZE = 20
RED_MIN_CLUSTER_HITS = 7

CIRCLE_STEP_RAD = 0.045
CIRCLE_MOTION_DEADBAND_PX = 3
CIRCLE_RIGHT_SWITCH_HOLD_MS = 160

LOCK_CONFIRM_FRAMES = 1
LOCK_MAX_JUMP = 95
LOCK_SOFT_JUMP = 55
PENDING_MAX_JUMP = 32
LOST_HOLD_FRAMES = 2
UART_MIN_PERIOD_MS = 18
GC_EVERY_N = 32

global_theta = 0.0
circle_dir = -1
circle_hold_until_ms = 0
circle_last_anchor_x = None


def format_coord(coord):
    if coord > 999:
        coord = 999
    if coord < -999:
        coord = -999
    return "%+04d" % coord


def send_uart_packet(dx, dy):
    uart.write("[%s%s*]" % (format_coord(dx), format_coord(dy)))


def send_uart_limited(dx, dy, now_ms, last_send_ms):
    if time.ticks_diff(now_ms, last_send_ms) >= UART_MIN_PERIOD_MS:
        send_uart_packet(dx, dy)
        return now_ms
    return last_send_ms


def dist2(x1, y1, x2, y2):
    dx = x1 - x2
    dy = y1 - y2
    return dx * dx + dy * dy


def line_intersection(x1, y1, x2, y2, x3, y3, x4, y4):
    abx = x2 - x1
    aby = y2 - y1
    acx = x3 - x1
    acy = y3 - y1
    cdx = x4 - x3
    cdy = y4 - y3
    det = abx * cdy - aby * cdx
    if det == 0:
        return None
    t = (acx * cdy - acy * cdx) / det
    return int(x1 + t * abx), int(y1 + t * aby)


def canonicalize_corners(x0, y0, x1, y1, x2, y2, x3, y3):
    pts = ((x0, y0), (x1, y1), (x2, y2), (x3, y3))
    tl = pts[0]
    tr = pts[0]
    br = pts[0]
    bl = pts[0]

    min_sum = x0 + y0
    max_sum = x0 + y0
    max_diff = x0 - y0
    min_diff = x0 - y0

    for p in pts:
        s = p[0] + p[1]
        d = p[0] - p[1]
        if s < min_sum:
            min_sum = s
            tl = p
        if s > max_sum:
            max_sum = s
            br = p
        if d > max_diff:
            max_diff = d
            tr = p
        if d < min_diff:
            min_diff = d
            bl = p

    return tl[0], tl[1], tr[0], tr[1], br[0], br[1], bl[0], bl[1]


def is_dark_pixel(img, x, y):
    if x < 2 or x >= IMG_W - 2 or y < 2 or y >= IMG_H - 2:
        return False
    p = img.get_pixel(int(x), int(y))
    if p is None:
        return False
    if isinstance(p, tuple):
        return (p[0] + p[1] + p[2]) < EDGE_BLACK_SUM
    return p < EDGE_BLACK_SUM


def is_red_pixel_value(p):
    if p is None or not isinstance(p, tuple):
        return False
    r = p[0]
    g = p[1]
    b = p[2]
    return r > 90 and r > g + 35 and r > b + 18


def is_target_red_pixel(p):
    if p is None or not isinstance(p, tuple):
        return False
    r = p[0]
    g = p[1]
    b = p[2]
    # More permissive than refine_center_by_red(), because this runs only after
    # rectangle detection failed. It catches dim red rings under indoor light.
    return r > 55 and r > g + 14 and r > b + 12 and (r - min(g, b)) > 24


def find_red_center_fallback(img, last_cx, last_cy, locked):
    if not USE_RED_CENTER_FALLBACK:
        return None

    cols = (IMG_W + RED_GRID_SIZE - 1) // RED_GRID_SIZE
    rows = (IMG_H + RED_GRID_SIZE - 1) // RED_GRID_SIZE
    size = cols * rows
    counts = [0] * size
    sums_x = [0] * size
    sums_y = [0] * size

    for y in range(8, IMG_H - 8, RED_FALLBACK_STEP):
        row = (y // RED_GRID_SIZE) * cols
        for x in range(8, IMG_W - 8, RED_FALLBACK_STEP):
            if is_target_red_pixel(img.get_pixel(x, y)):
                idx = row + (x // RED_GRID_SIZE)
                counts[idx] += 1
                sums_x[idx] += x
                sums_y[idx] += y

    best_score = -999999
    best = None
    for gy in range(rows):
        for gx in range(cols):
            total = 0
            sx = 0
            sy = 0
            for yy in range(gy - 1, gy + 2):
                if yy < 0 or yy >= rows:
                    continue
                base = yy * cols
                for xx in range(gx - 1, gx + 2):
                    if xx < 0 or xx >= cols:
                        continue
                    idx = base + xx
                    total += counts[idx]
                    sx += sums_x[idx]
                    sy += sums_y[idx]

            if total < RED_MIN_CLUSTER_HITS:
                continue

            cx = int(sx / total)
            cy = int(sy / total)
            if locked:
                score = total * 120 - sqrt(dist2(cx, cy, last_cx, last_cy)) * 6
            else:
                score = total * 120 - sqrt(dist2(cx, cy, IMG_CX, IMG_CY)) * 0.9

            if score > best_score:
                best_score = score
                best = (cx, cy, total)

    return best


def count_dark_edges(img, x0, y0, x1, y1, x2, y2, x3, y3):
    hits = 0
    if is_dark_pixel(img, (x0 + x1) * 0.5, (y0 + y1) * 0.5):
        hits += 1
    if is_dark_pixel(img, (x1 + x2) * 0.5, (y1 + y2) * 0.5):
        hits += 1
    if is_dark_pixel(img, (x2 + x3) * 0.5, (y2 + y3) * 0.5):
        hits += 1
    if is_dark_pixel(img, (x3 + x0) * 0.5, (y3 + y0) * 0.5):
        hits += 1
    return hits


def native_find_rects_as_cv_lite(img):
    if not USE_NATIVE_FIND_RECTS_FALLBACK:
        return None
    if not hasattr(img, "find_rects"):
        return None

    try:
        native_rects = img.find_rects(threshold=9000)
    except BaseException:
        return None

    rects = []
    for nr in native_rects:
        try:
            corners = nr.corners()
        except BaseException:
            continue
        if not corners or len(corners) < 4:
            continue

        x0 = int(corners[0][0])
        y0 = int(corners[0][1])
        x1 = int(corners[1][0])
        y1 = int(corners[1][1])
        x2 = int(corners[2][0])
        y2 = int(corners[2][1])
        x3 = int(corners[3][0])
        y3 = int(corners[3][1])

        min_x = min(x0, x1, x2, x3)
        max_x = max(x0, x1, x2, x3)
        min_y = min(y0, y1, y2, y3)
        max_y = max(y0, y1, y2, y3)
        w = max_x - min_x
        h = max_y - min_y
        if w <= 0 or h <= 0:
            continue

        rects.append([min_x, min_y, w, h, x0, y0, x1, y1, x2, y2, x3, y3])

    return rects


def choose_best_rect(rects, img, area_threshold, last_cx, last_cy, locked):
    best = None
    best_score = -999999
    loose_best = None
    loose_score_best = -999999
    max_area = IMG_W * IMG_H * MAX_AREA_RATIO

    for r in rects:
        box_area = r[2] * r[3]
        if box_area < area_threshold or box_area > max_area:
            continue

        x0 = r[4]
        y0 = r[5]
        x1 = r[6]
        y1 = r[7]
        x2 = r[8]
        y2 = r[9]
        x3 = r[10]
        y3 = r[11]
        x0, y0, x1, y1, x2, y2, x3, y3 = canonicalize_corners(x0, y0, x1, y1, x2, y2, x3, y3)

        inter = line_intersection(x0, y0, x2, y2, x1, y1, x3, y3)
        if inter is None:
            continue
        cx, cy = inter
        if cx < 0 or cx >= IMG_W or cy < 0 or cy >= IMG_H:
            continue

        l01 = sqrt(dist2(x0, y0, x1, y1))
        l12 = sqrt(dist2(x1, y1, x2, y2))
        l23 = sqrt(dist2(x2, y2, x3, y3))
        l30 = sqrt(dist2(x3, y3, x0, y0))
        if l01 < MIN_SIDE_PX or l12 < MIN_SIDE_PX or l23 < MIN_SIDE_PX or l30 < MIN_SIDE_PX:
            continue

        side_a = (l01 + l23) * 0.5
        side_b = (l12 + l30) * 0.5
        if side_a <= 0 or side_b <= 0:
            continue
        ratio = side_a / side_b
        if ratio < 1.0:
            ratio = 1.0 / ratio

        dark_hits = count_dark_edges(img, x0, y0, x1, y1, x2, y2, x3, y3)
        center_err = sqrt(dist2(cx, cy, IMG_CX, IMG_CY))

        loose_score = min(box_area, 26000) * 0.02 - center_err * 0.65 - abs(ratio - A4_RATIO) * 90 + dark_hits * 90
        if locked:
            jump = sqrt(dist2(cx, cy, last_cx, last_cy))
            if jump > LOCK_MAX_JUMP:
                loose_score -= 2000 + jump * 8
            else:
                loose_score += 240 - jump * 2
        if USE_LOOSE_RECT_FALLBACK and ratio <= 3.2 and loose_score > loose_score_best:
            loose_score_best = loose_score
            loose_best = (x0, y0, x1, y1, x2, y2, x3, y3, cx, cy, box_area, ratio, dark_hits)

        if ratio < ASPECT_MIN or ratio > ASPECT_MAX:
            continue
        if dark_hits < MIN_DARK_EDGE_HITS:
            continue

        aspect_err = abs(ratio - A4_RATIO)
        score = 900 - aspect_err * 700 + dark_hits * 180 + min(box_area, 22000) * 0.018

        if locked:
            jump = sqrt(dist2(cx, cy, last_cx, last_cy))
            if jump > LOCK_MAX_JUMP:
                continue
            if jump > LOCK_SOFT_JUMP:
                score -= (jump - LOCK_SOFT_JUMP) * 12
            else:
                score += 260 - jump * 2
        else:
            score -= center_err * 0.35

        if score > best_score:
            best_score = score
            best = (x0, y0, x1, y1, x2, y2, x3, y3, cx, cy, box_area, ratio, dark_hits)

    if best:
        return best
    return loose_best


def update_circle_theta(anchor_cx, now_ms):
    global global_theta, circle_dir, circle_hold_until_ms, circle_last_anchor_x

    if circle_last_anchor_x is None:
        circle_last_anchor_x = anchor_cx

    move_x = anchor_cx - circle_last_anchor_x
    desired_dir = circle_dir

    if move_x < -CIRCLE_MOTION_DEADBAND_PX:
        # Target moved left on screen: draw counterclockwise.
        desired_dir = -1
        circle_last_anchor_x = anchor_cx
    elif move_x > CIRCLE_MOTION_DEADBAND_PX:
        # Target moved right on screen: switch to clockwise, but hold phase first.
        desired_dir = 1
        circle_last_anchor_x = anchor_cx

    if desired_dir != circle_dir:
        circle_dir = desired_dir
        if circle_dir > 0:
            circle_hold_until_ms = now_ms + CIRCLE_RIGHT_SWITCH_HOLD_MS

    if circle_hold_until_ms and time.ticks_diff(now_ms, circle_hold_until_ms) < 0:
        return

    circle_hold_until_ms = 0
    global_theta += CIRCLE_STEP_RAD * circle_dir
    if global_theta > 2 * math.pi:
        global_theta -= 2 * math.pi
    elif global_theta < 0:
        global_theta += 2 * math.pi


def circle_motion_label(now_ms):
    if circle_hold_until_ms and time.ticks_diff(now_ms, circle_hold_until_ms) < 0:
        return "CW-HOLD" if circle_dir > 0 else "CCW-HOLD"
    return "CW" if circle_dir > 0 else "CCW"


def project_circle_target(corners, raw_cx, raw_cy, now_ms):
    x0, y0, x1, y1, x2, y2, x3, y3 = corners

    vec_wx = ((x1 - x0) + (x2 - x3)) * 0.5
    vec_wy = ((y1 - y0) + (y2 - y3)) * 0.5
    vec_hx = ((x3 - x0) + (x2 - x1)) * 0.5
    vec_hy = ((y3 - y0) + (y2 - y1)) * 0.5

    len_w = math.sqrt(vec_wx * vec_wx + vec_wy * vec_wy)
    len_h = math.sqrt(vec_hx * vec_hx + vec_hy * vec_hy)
    if len_w <= 1 or len_h <= 1:
        return raw_cx, raw_cy

    if len_w < len_h:
        short_x, short_y = vec_wx, vec_wy
        long_x, long_y = vec_hx, vec_hy
    else:
        short_x, short_y = vec_hx, vec_hy
        long_x, long_y = vec_wx, vec_wy

    ratio_short = TARGET_RADIUS_CM / A4_SHORT_CM
    ratio_long = TARGET_RADIUS_CM / A4_LONG_CM

    update_circle_theta(raw_cx, now_ms)

    co = math.cos(global_theta)
    si = math.sin(global_theta)
    tx = int(raw_cx + ratio_short * short_x * co + ratio_long * long_x * si)
    ty = int(raw_cy + ratio_short * short_y * co + ratio_long * long_y * si)
    return tx, ty


def refine_center_by_red(img, corners, cx, cy):
    if not REFINE_WITH_RED_CENTER:
        return cx, cy, 0

    x0, y0, x1, y1, x2, y2, x3, y3 = corners
    s1 = sqrt(dist2(x0, y0, x1, y1))
    s2 = sqrt(dist2(x1, y1, x2, y2))
    s3 = sqrt(dist2(x2, y2, x3, y3))
    s4 = sqrt(dist2(x3, y3, x0, y0))
    short_side = min(s1, s2, s3, s4)
    half = int(short_side * 0.35)
    if half < 12:
        half = 12
    if half > 58:
        half = 58

    x_start = max(2, cx - half)
    x_end = min(IMG_W - 2, cx + half)
    y_start = max(2, cy - half)
    y_end = min(IMG_H - 2, cy + half)

    sx = 0
    sy = 0
    count = 0
    for y in range(y_start, y_end, RED_SCAN_STEP):
        for x in range(x_start, x_end, RED_SCAN_STEP):
            if is_red_pixel_value(img.get_pixel(x, y)):
                sx += x
                sy += y
                count += 1

    if count >= RED_MIN_HITS:
        return int(sx / count), int(sy / count), count
    return cx, cy, 0


def draw_overlay(img, target, dx, dy, fps_value, area_threshold, rect_count):
    if not DRAW_DEBUG:
        return

    x0, y0, x1, y1, x2, y2, x3, y3, raw_cx, raw_cy, area, ratio, dark_hits = target
    img.draw_line(x0, y0, x1, y1, color=(0, 255, 0), thickness=2)
    img.draw_line(x1, y1, x2, y2, color=(0, 255, 0), thickness=2)
    img.draw_line(x2, y2, x3, y3, color=(0, 255, 0), thickness=2)
    img.draw_line(x3, y3, x0, y0, color=(0, 255, 0), thickness=2)
    img.draw_circle(raw_cx, raw_cy, 3, color=(255, 0, 0), fill=True)
    img.draw_string_advanced(0, 0, 26, "DX:%d DY:%d" % (dx, dy), color=(255, 255, 0))
    img.draw_string_advanced(210, 0, 26, "FPS:%d" % int(fps_value), color=(0, 255, 0))
    img.draw_string_advanced(0, 30, 18, "A:%d Q:%d R:%.2f B:%d T:%d" % (area, rect_count, ratio, dark_hits, area_threshold), color=(0, 255, 255))


def draw_projected_circle_points(img, corners, raw_cx, raw_cy):
    if not DRAW_PROJECTED_CIRCLE_POINTS:
        return

    x0, y0, x1, y1, x2, y2, x3, y3 = corners
    vec_wx = ((x1 - x0) + (x2 - x3)) * 0.5
    vec_wy = ((y1 - y0) + (y2 - y3)) * 0.5
    vec_hx = ((x3 - x0) + (x2 - x1)) * 0.5
    vec_hy = ((y3 - y0) + (y2 - y1)) * 0.5
    len_w = math.sqrt(vec_wx * vec_wx + vec_wy * vec_wy)
    len_h = math.sqrt(vec_hx * vec_hx + vec_hy * vec_hy)
    if len_w <= 1 or len_h <= 1:
        return
    if len_w < len_h:
        short_x, short_y = vec_wx, vec_wy
        long_x, long_y = vec_hx, vec_hy
    else:
        short_x, short_y = vec_hx, vec_hy
        long_x, long_y = vec_wx, vec_wy

    ratio_short = TARGET_RADIUS_CM / A4_SHORT_CM
    ratio_long = TARGET_RADIUS_CM / A4_LONG_CM
    for deg in range(0, 360, 20):
        rad = math.radians(deg)
        px = int(raw_cx + ratio_short * short_x * math.cos(rad) + ratio_long * long_x * math.sin(rad))
        py = int(raw_cy + ratio_short * short_y * math.cos(rad) + ratio_long * long_y * math.sin(rad))
        img.draw_circle(px, py, 1, color=(0, 200, 255), fill=True)


def camera_init():
    global sensor
    sensor = Sensor()
    sensor.reset()
    sensor.set_framesize(width=IMG_W, height=IMG_H)
    sensor.set_pixformat(Sensor.RGB888)

    Display.init(Display.ST7701, width=DETECT_WIDTH, height=DETECT_HEIGHT, fps=100, to_ide=True)
    MediaManager.init()
    sensor.run()


def camera_deinit():
    global sensor
    sensor.stop()
    Display.deinit()
    os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
    time.sleep_ms(100)
    MediaManager.deinit()


def capture_picture():
    fps = time.clock()
    area_threshold = 1500
    last_key_time = 0
    last_uart_time = -1000
    debounce_delay = 200

    frame_id = 0
    has_lock = False
    lost_frames = 99
    pending_cx = -1000
    pending_cy = -1000
    pending_count = 0
    last_cx = IMG_CX
    last_cy = IMG_CY
    last_dx = 999
    last_dy = 999

    while True:
        fps.tick()
        frame_id += 1
        now = ticks_ms()
        display_this_frame = (frame_id % DISPLAY_EVERY_N) == 0

        try:
            os.exitpoint()
            img = sensor.snapshot()
            img_np = img.to_numpy_ref()

            rects = cv_lite.rgb888_find_rectangles_with_corners(
                image_shape,
                img_np,
                canny_thresh1,
                canny_thresh2,
                approx_epsilon,
                area_min_ratio,
                max_angle_cos,
                gaussian_blur_size,
            )
            if not rects:
                rects = native_find_rects_as_cv_lite(img)
            rect_count = len(rects) if rects else 0

            if INC_KEY.value() == 0 and time.ticks_diff(now, last_key_time) > debounce_delay:
                area_threshold = min(12000, area_threshold + 300)
                last_key_time = now
            if DEC_KEY.value() == 0 and time.ticks_diff(now, last_key_time) > debounce_delay:
                area_threshold = max(600, area_threshold - 300)
                last_key_time = now

            accepted = None
            if rects:
                best = choose_best_rect(rects, img, area_threshold, last_cx, last_cy, has_lock)
                if best:
                    cx = best[8]
                    cy = best[9]

                    if has_lock:
                        accepted = best
                    else:
                        if dist2(cx, cy, pending_cx, pending_cy) <= PENDING_MAX_JUMP * PENDING_MAX_JUMP:
                            pending_count += 1
                        else:
                            pending_count = 1
                        pending_cx = cx
                        pending_cy = cy
                        if pending_count >= LOCK_CONFIRM_FRAMES:
                            accepted = best
                            has_lock = True

            red_fallback = None
            if not accepted:
                red_fallback = find_red_center_fallback(img, last_cx, last_cy, has_lock)

            if accepted:
                raw_cx = accepted[8]
                raw_cy = accepted[9]
                corners = accepted[0:8]
                raw_cx, raw_cy, red_hits = refine_center_by_red(img, corners, raw_cx, raw_cy)
                target_x = raw_cx
                target_y = raw_cy

                if DRAW_CIRCLE_MODE:
                    target_x, target_y = project_circle_target(corners, raw_cx, raw_cy, now)

                dx = IMG_CX - target_x
                dy = IMG_CY - target_y
                if -2 <= dx <= 2:
                    dx = 0
                if -2 <= dy <= 2:
                    dy = 0

                last_uart_time = send_uart_limited(dx, dy, now, last_uart_time)
                last_dx = dx
                last_dy = dy
                last_cx = raw_cx
                last_cy = raw_cy
                lost_frames = 0

                if display_this_frame:
                    draw_projected_circle_points(img, corners, raw_cx, raw_cy)
                    if DRAW_CIRCLE_MODE:
                        img.draw_circle(target_x, target_y, 6, color=(255, 0, 255), thickness=3)
                    draw_overlay(img, accepted, dx, dy, fps.fps(), area_threshold, rect_count)
                    if DRAW_DEBUG and red_hits > 0:
                        img.draw_circle(raw_cx, raw_cy, 5, color=(255, 80, 80), thickness=2)
                        img.draw_string_advanced(0, 52, 18, "RED:%d" % red_hits, color=(255, 80, 80))
                    if DRAW_DEBUG and DRAW_CIRCLE_MODE:
                        img.draw_string_advanced(0, 72, 18, "DIR:%s" % circle_motion_label(now), color=(255, 180, 0))
                    Display.show_image(img)
            elif red_fallback:
                raw_cx = red_fallback[0]
                raw_cy = red_fallback[1]
                red_hits = red_fallback[2]

                dx = IMG_CX - raw_cx
                dy = IMG_CY - raw_cy
                if -2 <= dx <= 2:
                    dx = 0
                if -2 <= dy <= 2:
                    dy = 0

                last_uart_time = send_uart_limited(dx, dy, now, last_uart_time)
                last_dx = dx
                last_dy = dy
                last_cx = raw_cx
                last_cy = raw_cy
                has_lock = True
                pending_count = 1
                lost_frames = 0

                if display_this_frame:
                    if DRAW_DEBUG:
                        img.draw_circle(raw_cx, raw_cy, 7, color=(255, 0, 255), thickness=3)
                        img.draw_string_advanced(8, 0, 26, "RED DX:%d DY:%d" % (dx, dy), color=(255, 255, 0))
                        img.draw_string_advanced(8, 32, 22, "FPS:%d H:%d Q:%d" % (int(fps.fps()), red_hits, rect_count), color=(0, 255, 0))
                    Display.show_image(img)
            else:
                lost_frames += 1
                if lost_frames <= LOST_HOLD_FRAMES and has_lock:
                    last_uart_time = send_uart_limited(last_dx, last_dy, now, last_uart_time)
                else:
                    has_lock = False
                    pending_count = 0
                    last_uart_time = send_uart_limited(999, 999, now, last_uart_time)

                if display_this_frame:
                    if DRAW_DEBUG:
                        img.draw_string_advanced(8, 0, 26, "LOST/ACQ  T:%d Q:%d" % (area_threshold, rect_count), color=(255, 0, 0))
                        img.draw_string_advanced(8, 32, 22, "FPS:%d  P:%d" % (int(fps.fps()), pending_count), color=(0, 255, 0))
                    Display.show_image(img)

            if (frame_id & (GC_EVERY_N - 1)) == 0:
                gc.collect()

        except KeyboardInterrupt:
            print("User stopped.")
            break
        except BaseException as e:
            # Rate-limit recovery. Frequent printing/display is another source of stalls.
            print("Loop exception:", e)
            time.sleep_ms(20)
            gc.collect()


def main():
    os.exitpoint(os.EXITPOINT_ENABLE)
    camera_is_init = False
    try:
        camera_init()
        camera_is_init = True
        capture_picture()
    except Exception as e:
        print("Exception", e)
    finally:
        if camera_is_init:
            camera_deinit()


if __name__ == "__main__":
    main()
