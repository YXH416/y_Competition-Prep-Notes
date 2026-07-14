# 简易自行瞄准装置：K230/CanMV 靶面矩形检测与串口输出
import time
import os
import math

from media.sensor import *
from media.display import *
from media.media import *
import cv_lite
from time import ticks_ms
from machine import UART, FPIOA, Pin


# ========================= 硬件与通信配置 =========================
fpioa = FPIOA()
fpioa.set_function(11, fpioa.UART2_TXD)
fpioa.set_function(12, fpioa.UART2_RXD)
fpioa.set_function(32, FPIOA.GPIO32)
fpioa.set_function(33, FPIOA.GPIO33)

INC_KEY = Pin(32, Pin.IN, Pin.PULL_UP)  # 增大面积阈值
DEC_KEY = Pin(33, Pin.IN, Pin.PULL_UP)  # 减小面积阈值

uart = UART(
    UART.UART2,
    baudrate=115200,
    bits=UART.EIGHTBITS,
    parity=UART.PARITY_NONE,
    stop=UART.STOPBITS_ONE,
)


# ========================= 图像与检测参数 =========================
FRAME_WIDTH = 480
FRAME_HEIGHT = 320
IMAGE_SHAPE = [FRAME_HEIGHT, FRAME_WIDTH]  # cv_lite 使用 [高, 宽]
IMAGE_CENTER_X = FRAME_WIDTH // 2
IMAGE_CENTER_Y = FRAME_HEIGHT // 2

DETECT_WIDTH = ALIGN_UP(800, 16)
DETECT_HEIGHT = 480

# 若激光光轴与画面中心存在固定偏差，在这里标定，单位为像素。
# 正值会使期望瞄准点向图像右/下移动。
AIM_OFFSET_X = 0
AIM_OFFSET_Y = 0

# cv_lite 矩形检测参数
CANNY_THRESH1 = 50
CANNY_THRESH2 = 150
APPROX_EPSILON = 0.04
AREA_MIN_RATIO = 0.001
MAX_ANGLE_COS = 0.5
GAUSSIAN_BLUR_SIZE = 5

# 主检测失败后，仅在已有跟踪记忆时启用更敏感的备用检测。
FALLBACK_CANNY_THRESH1 = 25
FALLBACK_CANNY_THRESH2 = 100
FALLBACK_APPROX_EPSILON = 0.05
FALLBACK_MAX_ANGLE_COS = 0.6
FALLBACK_GAUSSIAN_BLUR_SIZE = 3

# 靶框几何筛选参数。采用比例误差，避免分辨率/距离变化导致误判。
INITIAL_AREA_THRESHOLD = 2000
AREA_THRESHOLD_MIN = 500
AREA_THRESHOLD_MAX = 10000
AREA_THRESHOLD_STEP = 500
MIN_SIDE_LENGTH = 30
TRACK_AREA_THRESHOLD = 1000
TRACK_MIN_SIDE_LENGTH = 20
MAX_OPPOSITE_SIDE_ERROR = 0.60
# 目标靶可能被斜视，透视投影下物理平行边在图像中会明显会聚。
# 凸性与四个角的直角误差会共同限制放宽平行误差后产生的伪框。
MAX_PARALLEL_ERROR_DEG = 45.0
MAX_RIGHT_ANGLE_ERROR_DEG = 30.0

KEY_DEBOUNCE_MS = 200
UART_DIAG_INTERVAL_MS = 500
INVALID_COORD = 999

# 实测约 64 FPS；保持 5 帧约为 78 ms，只跨越瞬时运动模糊，不会长时间盲控。
LOST_HOLD_FRAMES = 5
TRACK_MEMORY_FRAMES = 30
TRACK_GATE_BASE_PX = 70
TRACK_GATE_GROWTH_PX = 5
TRACK_GATE_MAX_PX = 200
CENTER_FILTER_ALPHA = 0.55
FAST_FILTER_ALPHA = 0.85
FAST_MOVE_THRESHOLD_PX = 20
FILTER_RESET_JUMP_PX = 80

sensor = None


# ========================= 串口协议 =========================
# 控制帧（保持原协议兼容）：[+xxx-yyy*]
#   dx = 期望点x - 靶心x；dy = 期望点y - 靶心y
#   dx > 0 表示靶心在画面中心左侧；dy > 0 表示靶心在画面中心上方。
#   丢靶时发送：[+999+999*]
# 诊断行：
#   #CFG,...  启动参数或按键修改参数
#   #DBG,...  周期检测数据
#   #ERR,...  未捕获异常

def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def format_coord(coord):
    value = clamp(int(round(coord)), -INVALID_COORD, INVALID_COORD)
    return "%+04d" % value


def make_control_packet(dx, dy, locked):
    if not locked:
        dx = INVALID_COORD
        dy = INVALID_COORD
    return "[%s%s*]" % (format_coord(dx), format_coord(dy))


def uart_print(line):
    """同时送到 UART2 和 IDE 串行终端，诊断行以换行结束。"""
    # 控制帧没有换行；诊断前补换行，保证串口助手中 #CFG/#DBG/#ERR 独占一行。
    uart.write("\r\n" + line + "\r\n")
    print(line)


def report_config(area_threshold, event="BOOT"):
    uart_print(
        "#CFG,event=%s,size=%dx%d,aim=%d/%d,area_th=%d,min_side=%d,"
        "track_area=%d,track_side=%d,opp_max=%.2f,parallel_max=%.1f,"
        "right_max=%.1f,hold=%d,memory=%d,gate=%d+%d/%d,"
        "alpha=%.2f/%.2f@%d,reset_px=%d,fb=%d/%d/%.2f/%.1f/%d,diag_ms=%d"
        % (
            event,
            FRAME_WIDTH,
            FRAME_HEIGHT,
            IMAGE_CENTER_X + AIM_OFFSET_X,
            IMAGE_CENTER_Y + AIM_OFFSET_Y,
            area_threshold,
            MIN_SIDE_LENGTH,
            TRACK_AREA_THRESHOLD,
            TRACK_MIN_SIDE_LENGTH,
            MAX_OPPOSITE_SIDE_ERROR,
            MAX_PARALLEL_ERROR_DEG,
            MAX_RIGHT_ANGLE_ERROR_DEG,
            LOST_HOLD_FRAMES,
            TRACK_MEMORY_FRAMES,
            TRACK_GATE_BASE_PX,
            TRACK_GATE_GROWTH_PX,
            TRACK_GATE_MAX_PX,
            CENTER_FILTER_ALPHA,
            FAST_FILTER_ALPHA,
            FAST_MOVE_THRESHOLD_PX,
            FILTER_RESET_JUMP_PX,
            FALLBACK_CANNY_THRESH1,
            FALLBACK_CANNY_THRESH2,
            FALLBACK_APPROX_EPSILON,
            FALLBACK_MAX_ANGLE_COS,
            FALLBACK_GAUSSIAN_BLUR_SIZE,
            UART_DIAG_INTERVAL_MS,
        )
    )


def report_debug(
    sequence,
    state,
    detection_state,
    detector_source,
    candidate_count,
    valid_count,
    metrics,
    raw_center,
    control_center,
    dx,
    dy,
    stable_frames,
    lost_frames,
    dropout_events,
    missed_frames,
    fallback_attempts,
    fallback_locks,
    fail_no_rect,
    fail_area,
    fail_geometry,
    fail_jump,
    fps_value,
    process_ms,
    area_threshold,
    center_gate,
):
    if metrics is None:
        area = -1
        opposite_error_1 = -1.0
        opposite_error_2 = -1.0
        parallel_error_1 = -1.0
        parallel_error_2 = -1.0
        right_angle_error = -1.0
    else:
        area = metrics["area"]
        opposite_error_1 = metrics["opp1"]
        opposite_error_2 = metrics["opp2"]
        parallel_error_1 = metrics["par1"]
        parallel_error_2 = metrics["par2"]
        right_angle_error = metrics["right"]

    if raw_center is None:
        raw_x = -1
        raw_y = -1
    else:
        raw_x, raw_y = raw_center

    if control_center is None:
        control_x = -1
        control_y = -1
    else:
        control_x, control_y = control_center

    detection_rate = 100.0 * (sequence - missed_frames) / max(sequence, 1)

    uart_print(
        "#DBG,seq=%d,state=%s,reason=%s,src=%s,n=%d,valid=%d,area=%d,th=%d,gate=%d,"
        "opp=%.2f/%.2f,par=%.1f/%.1f,right=%.1f,raw=%d/%d,"
        "ctrl=%d/%d,err=%s/%s,stable=%d,lost=%d,drop=%d,miss=%d,"
        "rate=%.1f,fb=%d/%d,fail_nr=%d,fail_area=%d,fail_geo=%d,"
        "fail_jump=%d,fps=%d,ms=%d"
        % (
            sequence,
            state,
            detection_state,
            detector_source,
            candidate_count,
            valid_count,
            area,
            area_threshold,
            center_gate,
            opposite_error_1,
            opposite_error_2,
            parallel_error_1,
            parallel_error_2,
            right_angle_error,
            raw_x,
            raw_y,
            control_x,
            control_y,
            format_coord(dx),
            format_coord(dy),
            stable_frames,
            lost_frames,
            dropout_events,
            missed_frames,
            detection_rate,
            fallback_locks,
            fallback_attempts,
            fail_no_rect,
            fail_area,
            fail_geometry,
            fail_jump,
            int(fps_value),
            process_ms,
        )
    )


def report_error(stage, error):
    message = str(error).replace(",", ";").replace("\r", " ").replace("\n", " ")
    uart_print("#ERR,stage=%s,type=%s,msg=%s" % (stage, type(error).__name__, message))


# ========================= 几何工具 =========================
def point_distance(point1, point2):
    dx = point1[0] - point2[0]
    dy = point1[1] - point2[1]
    return math.sqrt(dx * dx + dy * dy)


def update_filtered_center(raw_center, previous_center):
    """自适应指数滤波：小移动抑制抖动，大移动快速跟随。"""
    raw_x, raw_y = raw_center
    if previous_center is None:
        return float(raw_x), float(raw_y)

    move_distance = point_distance(raw_center, previous_center)
    if move_distance > FILTER_RESET_JUMP_PX:
        return float(raw_x), float(raw_y)

    alpha = FAST_FILTER_ALPHA if move_distance > FAST_MOVE_THRESHOLD_PX else CENTER_FILTER_ALPHA
    return (
        alpha * raw_x + (1.0 - alpha) * previous_center[0],
        alpha * raw_y + (1.0 - alpha) * previous_center[1],
    )


def order_corners(corners):
    """将四个角点按环绕中心的顺序排列，消除检测库角点顺序差异。"""
    center_x = sum(point[0] for point in corners) / 4.0
    center_y = sum(point[1] for point in corners) / 4.0
    return sorted(corners, key=lambda point: math.atan2(point[1] - center_y, point[0] - center_x))


def line_angle_deg(point1, point2):
    angle = math.degrees(m；ath.atan2(point2[1] - point1[1], point2[0] - point1[0]))
    return angle % 180.0


def angle_difference_180(angle1, angle2):
    """无方向直线夹角，返回 0~90 度。"""
    difference = abs(angle1 - angle2) % 180.0
    return min(difference, 180.0 - difference)


def find_intersection(point1, point2, point3, point4):
    """求两条直线交点；近平行时返回 None，避免除零。"""
    x1, y1 = point1
    x2, y2 = point2
    x3, y3 = point3
    x4, y4 = point4

    denominator = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    if abs(denominator) < 0.000001:
        return None

    determinant1 = x1 * y2 - y1 * x2
    determinant2 = x3 * y4 - y3 * x4
    center_x = (determinant1 * (x3 - x4) - (x1 - x2) * determinant2) / denominator
    center_y = (determinant1 * (y3 - y4) - (y1 - y2) * determinant2) / denominator
    return int(round(center_x)), int(round(center_y))


def is_convex_quad(corners):
    """四个有序角点必须构成凸四边形，排除凹陷或自交伪轮廓。"""
    cross_sign = 0
    for index in range(4):
        point1 = corners[index]
        point2 = corners[(index + 1) % 4]
        point3 = corners[(index + 2) % 4]
        cross = (
            (point2[0] - point1[0]) * (point3[1] - point2[1])
            - (point2[1] - point1[1]) * (point3[0] - point2[0])
        )
        if abs(cross) < 1:
            return False
        current_sign = 1 if cross > 0 else -1
        if cross_sign == 0:
            cross_sign = current_sign
        elif current_sign != cross_sign:
            return False
    return True


def analyze_rectangle(rectangle, area_threshold, min_side_length=MIN_SIDE_LENGTH):
    """返回 (状态, 几何数据)。状态为 LOCK 时才可用于控制。"""
    if rectangle is None or len(rectangle) < 12:
        return "FORMAT", None

    try:
        corners = order_corners(
            [
                (int(rectangle[4]), int(rectangle[5])),
                (int(rectangle[6]), int(rectangle[7])),
                (int(rectangle[8]), int(rectangle[9])),
                (int(rectangle[10]), int(rectangle[11])),
            ]
        )
        area = int(abs(rectangle[2] * rectangle[3]))
    except Exception:
        return "FORMAT", None

    sides = [
        point_distance(corners[0], corners[1]),
        point_distance(corners[1], corners[2]),
        point_distance(corners[2], corners[3]),
        point_distance(corners[3], corners[0]),
    ]
    angles = [
        line_angle_deg(corners[0], corners[1]),
        line_angle_deg(corners[1], corners[2]),
        line_angle_deg(corners[2], corners[3]),
        line_angle_deg(corners[3], corners[0]),
    ]

    opposite_error_1 = abs(sides[0] - sides[2]) / max(sides[0], sides[2], 1.0)
    opposite_error_2 = abs(sides[1] - sides[3]) / max(sides[1], sides[3], 1.0)
    parallel_error_1 = angle_difference_180(angles[0], angles[2])
    parallel_error_2 = angle_difference_180(angles[1], angles[3])
    right_angle_error = max(
        abs(angle_difference_180(angles[index], angles[(index + 1) % 4]) - 90.0)
        for index in range(4)
    )
    center = find_intersection(corners[0], corners[2], corners[1], corners[3])

    metrics = {
        "corners": corners,
        "area": area,
        "opp1": opposite_error_1,
        "opp2": opposite_error_2,
        "par1": parallel_error_1,
        "par2": parallel_error_2,
        "right": right_angle_error,
        "center": center,
    }

    if area < area_threshold:
        return "AREA", metrics
    if not is_convex_quad(corners):
        return "CONCAVE", metrics
    if min(sides) < min_side_length:
        return "SIDE", metrics
    if opposite_error_1 > MAX_OPPOSITE_SIDE_ERROR or opposite_error_2 > MAX_OPPOSITE_SIDE_ERROR:
        return "OPPOSITE", metrics
    if parallel_error_1 > MAX_PARALLEL_ERROR_DEG or parallel_error_2 > MAX_PARALLEL_ERROR_DEG:
        return "PARALLEL", metrics
    if right_angle_error > MAX_RIGHT_ANGLE_ERROR_DEG:
        return "RIGHT_ANGLE", metrics
    if center is None:
        return "CENTER_NONE", metrics
    if not (0 <= center[0] < FRAME_WIDTH and 0 <= center[1] < FRAME_HEIGHT):
        return "CENTER_OUT", metrics

    return "LOCK", metrics


def select_target(
    rectangles,
    area_threshold,
    min_side_length=MIN_SIDE_LENGTH,
    reference_center=None,
    max_center_jump=None,
):
    """选最大有效框；跟踪时还需通过相对上一中心的距离门限。"""
    best_valid = None
    best_rejected = None
    rejected_state = "NO_RECT"
    valid_count = 0

    if not rectangles:
        return None, "NO_RECT", 0, 0, None

    candidate_count = len(rectangles)
    for rectangle in rectangles:
        state, metrics = analyze_rectangle(rectangle, area_threshold, min_side_length)
        if (
            state == "LOCK"
            and reference_center is not None
            and max_center_jump is not None
        ):
            track_distance = point_distance(metrics["center"], reference_center)
            metrics["track_distance"] = track_distance
            if track_distance > max_center_jump:
                state = "JUMP"

        if state == "LOCK":
            valid_count += 1
            if best_valid is None or metrics["area"] > best_valid["area"]:
                best_valid = metrics
        elif metrics is not None:
            if best_rejected is None or metrics["area"] > best_rejected["area"]:
                best_rejected = metrics
                rejected_state = state
        elif best_rejected is None:
            rejected_state = state

    if best_valid is not None:
        return best_valid, "LOCK", candidate_count, valid_count, best_rejected
    return None, rejected_state, candidate_count, 0, best_rejected


def draw_quad(image, corners, color, thickness=3):
    for index in range(4):
        next_index = (index + 1) % 4
        image.draw_line(
            corners[index][0],
            corners[index][1],
            corners[next_index][0],
            corners[next_index][1],
            color=color,
            thickness=thickness,
        )
        image.draw_circle(
            corners[index][0],
            corners[index][1],
            2,
            color=(0, 0, 255),
            fill=True,
            thickness=2,
        )


# ========================= 相机与主循环 =========================
def camera_init():
    global sensor
    sensor = Sensor()
    sensor.reset()
    sensor.set_framesize(width=FRAME_WIDTH, height=FRAME_HEIGHT)
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
    area_threshold = INITIAL_AREA_THRESHOLD
    last_key_time = 0
    last_diag_time = ticks_ms()
    sequence = 0
    stable_frames = 0
    lost_frames = 0
    dropout_events = 0
    missed_frames = 0
    fallback_attempts = 0
    fallback_locks = 0
    fail_no_rect = 0
    fail_area = 0
    fail_geometry = 0
    fail_jump = 0
    filtered_center = None

    report_config(area_threshold)

    while True:
        frame_start = ticks_ms()
        fps.tick()
        sequence += 1

        try:
            os.exitpoint()
            image = sensor.snapshot()
            image_np = image.to_numpy_ref()
            rectangles = cv_lite.rgb888_find_rectangles_with_corners(
                IMAGE_SHAPE,
                image_np,
                CANNY_THRESH1,
                CANNY_THRESH2,
                APPROX_EPSILON,
                AREA_MIN_RATIO,
                MAX_ANGLE_COS,
                GAUSSIAN_BLUR_SIZE,
            )

            current_time = ticks_ms()
            key_ready = time.ticks_diff(current_time, last_key_time) > KEY_DEBOUNCE_MS
            if key_ready and INC_KEY.value() == 0:
                area_threshold = min(AREA_THRESHOLD_MAX, area_threshold + AREA_THRESHOLD_STEP)
                last_key_time = current_time
                report_config(area_threshold, "KEY_INC")
            elif key_ready and DEC_KEY.value() == 0:
                area_threshold = max(AREA_THRESHOLD_MIN, area_threshold - AREA_THRESHOLD_STEP)
                last_key_time = current_time
                report_config(area_threshold, "KEY_DEC")

            tracking_active = (
                filtered_center is not None and lost_frames <= TRACK_MEMORY_FRAMES
            )
            if tracking_active:
                effective_area_threshold = min(area_threshold, TRACK_AREA_THRESHOLD)
                effective_min_side = TRACK_MIN_SIDE_LENGTH
                center_gate = min(
                    TRACK_GATE_MAX_PX,
                    TRACK_GATE_BASE_PX + lost_frames * TRACK_GATE_GROWTH_PX,
                )
                reference_center = filtered_center
            else:
                effective_area_threshold = area_threshold
                effective_min_side = MIN_SIDE_LENGTH
                center_gate = 0
                reference_center = None

            target, detection_state, candidate_count, valid_count, rejected = select_target(
                rectangles,
                effective_area_threshold,
                effective_min_side,
                reference_center,
                center_gate if tracking_active else None,
            )
            detector_source = "P" if target is not None else "-"

            # 已有跟踪记忆时，主检测失败才执行一次敏感备用检测。
            if target is None and tracking_active:
                fallback_attempts += 1
                fallback_rectangles = cv_lite.rgb888_find_rectangles_with_corners(
                    IMAGE_SHAPE,
                    image_np,
                    FALLBACK_CANNY_THRESH1,
                    FALLBACK_CANNY_THRESH2,
                    FALLBACK_APPROX_EPSILON,
                    AREA_MIN_RATIO,
                    FALLBACK_MAX_ANGLE_COS,
                    FALLBACK_GAUSSIAN_BLUR_SIZE,
                )
                (
                    fallback_target,
                    fallback_state,
                    fallback_candidate_count,
                    fallback_valid_count,
                    fallback_rejected,
                ) = select_target(
                    fallback_rectangles,
                    effective_area_threshold,
                    effective_min_side,
                    reference_center,
                    center_gate,
                )

                if fallback_target is not None:
                    target = fallback_target
                    detection_state = "LOCK"
                    candidate_count = fallback_candidate_count
                    valid_count = fallback_valid_count
                    rejected = fallback_rejected
                    detector_source = "F"
                    fallback_locks += 1
                elif detection_state == "NO_RECT" or fallback_state != "NO_RECT":
                    detection_state = fallback_state
                    candidate_count = fallback_candidate_count
                    valid_count = fallback_valid_count
                    rejected = fallback_rejected

            if target is None:
                if detection_state == "NO_RECT":
                    fail_no_rect += 1
                elif detection_state in ("AREA", "SIDE"):
                    fail_area += 1
                elif detection_state == "JUMP":
                    fail_jump += 1
                else:
                    fail_geometry += 1

            raw_center = None
            control_center = None
            dx = INVALID_COORD
            dy = INVALID_COORD
            output_locked = False
            control_state = "LOST"
            aim_x = IMAGE_CENTER_X + AIM_OFFSET_X
            aim_y = IMAGE_CENTER_Y + AIM_OFFSET_Y

            # 黄色圆为相机/激光标定后的期望点，始终显示，便于现场观察。
            image.draw_circle(aim_x, aim_y, 3, color=(255, 255, 0), fill=False, thickness=2)

            if target is not None:
                stable_frames += 1
                raw_center = target["center"]
                lost_frames = 0
                filtered_center = update_filtered_center(raw_center, filtered_center)
                control_center = (
                    int(round(filtered_center[0])),
                    int(round(filtered_center[1])),
                )
                dx = aim_x - control_center[0]
                dy = aim_y - control_center[1]
                output_locked = True
                control_state = "LOCK"

                draw_quad(image, target["corners"], (0, 255, 0), 3)
                image.draw_circle(raw_center[0], raw_center[1], 2, color=(255, 0, 0), fill=True, thickness=3)
                image.draw_circle(
                    control_center[0],
                    control_center[1],
                    4,
                    color=(0, 255, 255),
                    fill=False,
                    thickness=2,
                )
                image.draw_string_advanced(
                    5,
                    0,
                    24,
                    "LOCK/%s dx:%s dy:%s"
                    % (detector_source, format_coord(dx), format_coord(dy)),
                    color=(0, 255, 0),
                )
            else:
                stable_frames = 0
                missed_frames += 1

                if filtered_center is not None:
                    if lost_frames == 0:
                        dropout_events += 1
                    lost_frames += 1

                    if lost_frames <= LOST_HOLD_FRAMES:
                        control_center = (
                            int(round(filtered_center[0])),
                            int(round(filtered_center[1])),
                        )
                        dx = aim_x - control_center[0]
                        dy = aim_y - control_center[1]
                        output_locked = True
                        control_state = "HOLD"
                        image.draw_circle(
                            control_center[0],
                            control_center[1],
                            5,
                            color=(255, 165, 0),
                            fill=False,
                            thickness=2,
                        )
                        image.draw_string_advanced(
                            5,
                            0,
                            24,
                            "HOLD %d/%d dx:%s dy:%s"
                            % (
                                lost_frames,
                                LOST_HOLD_FRAMES,
                                format_coord(dx),
                                format_coord(dy),
                            ),
                            color=(255, 165, 0),
                        )
                    elif lost_frames > TRACK_MEMORY_FRAMES:
                        filtered_center = None
                        lost_frames = TRACK_MEMORY_FRAMES + 1
                else:
                    lost_frames = min(lost_frames + 1, TRACK_MEMORY_FRAMES + 1)

                if rejected is not None:
                    draw_quad(image, rejected["corners"], (255, 0, 0), 2)
                if not output_locked:
                    memory_mark = "MEM%d" % lost_frames if filtered_center is not None else "SEARCH"
                    image.draw_string_advanced(
                        5,
                        0,
                        24,
                        "LOST/%s %s" % (memory_mark, detection_state),
                        color=(255, 0, 0),
                    )

            # LOCK/HOLD 发送有效误差；连续丢失超过保持帧数后才发送 +999/+999。
            uart.write(make_control_packet(dx, dy, output_locked))

            image.draw_string_advanced(
                5,
                28,
                22,
                "N:%d V:%d S:%d/%d" % (
                    candidate_count,
                    valid_count,
                    target["area"] if target is not None else (
                        rejected["area"] if rejected is not None else 0
                    ),
                    effective_area_threshold,
                ),
                color=(255, 255, 0),
            )
            image.draw_string_advanced(
                5, 54, 22, "fps:%d" % int(fps.fps()), color=(0, 255, 0)
            )
            Display.show_image(image)

            process_ms = time.ticks_diff(ticks_ms(), frame_start)
            if time.ticks_diff(current_time, last_diag_time) >= UART_DIAG_INTERVAL_MS:
                diagnostic_metrics = target if target is not None else rejected
                report_debug(
                    sequence,
                    control_state,
                    detection_state,
                    detector_source,
                    candidate_count,
                    valid_count,
                    diagnostic_metrics,
                    raw_center,
                    control_center,
                    dx,
                    dy,
                    stable_frames,
                    lost_frames,
                    dropout_events,
                    missed_frames,
                    fallback_attempts,
                    fallback_locks,
                    fail_no_rect,
                    fail_area,
                    fail_geometry,
                    fail_jump,
                    fps.fps(),
                    process_ms,
                    effective_area_threshold,
                    center_gate,
                )
                last_diag_time = current_time

        except KeyboardInterrupt as error:
            report_error("USER_STOP", error)
            break
        except BaseException as error:
            report_error("CAPTURE", error)
            break


def main():
    os.exitpoint(os.EXITPOINT_ENABLE)
    camera_is_init = False
    try:
        camera_init()
        camera_is_init = True
        capture_picture()
    except BaseException as error:
        report_error("MAIN", error)
    finally:
        if camera_is_init:
            camera_deinit()


if __name__ == "__main__":
    main()
