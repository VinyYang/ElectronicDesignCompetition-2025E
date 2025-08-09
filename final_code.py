# -*- coding: utf-8 -*-
# ==================================================================================
# === OpenMV 简易自行瞄准装置 视觉控制整合程序 ===
# === 整合版 V1.9 (逻辑修正版) ===
# ==================================================================================
# 功能说明:
# 本脚本整合了多种工作模式的视觉支持功能：
# 1. 待机模式(0xA3) - 等待串口指令
# 2. 瞄准靶心模式(0xA1) - 使用黑色边框识别靶纸并计算中心点
# 3. 圆追踪模式(0xA2) - 沿着识别到的靶纸边缘进行圆周运动
#
# V1.9 更新日志:
# - [逻辑修正] 修正了主循环中的一个核心逻辑错误。现在，只要识别到目标，
#   视觉反馈（绘制矩形、十字等）会立即显示在图像上，不再与UART的发送频率挂钩。
#   这解决了在快速连续识别时，视觉反馈会因发送时间间隔未到而消失的问题。
# - [代码结构] 调整了瞄准模式和圆周模式下的代码块，将视觉反馈与通信逻辑分离，
#   使代码意图更清晰，行为更可靠。
# - [版本] 更新版本号至 V1.9。
#
# V1.8 更新日志:
# - [鲁棒性强化] 新增“边缘贴合检查”，有效防止因光照不均等问题将画面边角误识别为目标。
# ==================================================================================

# ----------------- 1. 导入所需模块 -----------------
import sensor
import image
import time
import struct
import gc
import math
from pyb import UART

# ----------------- 2. 初始化与全局配置 -----------------
# 初始化串口
uart = UART(3, 115200, timeout_char=100)

# 延迟时间配置
STANDARD_SEND_INTERVAL_MS = 30  # 标准模式延迟
CIRCLE_SEND_INTERVAL_MS = 50    # 圆周模式延迟
current_send_interval_ms = STANDARD_SEND_INTERVAL_MS
last_uart_send_time = 0

# 分辨率设置 (HQVGA: 240x160)
ACTIVE_RESOLUTION = sensor.HQVGA
SCREEN_WIDTH, SCREEN_HEIGHT = 240, 160
SCREEN_CENTER_X, SCREEN_CENTER_Y = SCREEN_WIDTH // 2, SCREEN_HEIGHT // 2

# 二值化参数
BLACK_THRESHOLD = (0, 20)      # 黑色区域阈值范围
BINARY_AREA_THRESHOLD = 2000   # 最小面积阈值
BINARY_PIXELS_THRESHOLD = 300  # 最小像素数阈值

# 圆轨迹控制参数
current_angle = 0           # 当前角度（弧度）
circle_start_time = None    # 圆周运动开始时间
target_circle_time = 15000  # 完成一圈的目标时间(毫秒)
last_target_point = None    # 上一个目标点
smooth_factor = 0.7         # 平滑因子，越大越平滑

# 椭圆参数
ellipse_a = None      # 椭圆长轴
ellipse_b = None      # 椭圆短轴
ellipse_angle = 0     # 椭圆旋转角度

# ----------------- 3. 定义状态与指令 -----------------
# 工作状态定义
STATE_IDLE = 0              # 待机状态
STATE_AIM_BULLSEYE = 1      # 瞄准靶心
STATE_CIRCLE_TRACKING = 2   # 圆追踪模式

# 命令字节定义
CMD_AIM_BULLSEYE = b'\xA1'    # 矩形检测（瞄准靶心）
CMD_CIRCLE_TRACKING = b'\xA2'  # 圆追踪检测
CMD_IDLE = b'\xA3'            # 待机模式

# 命令说明映射
COMMAND_MAP = {
    CMD_AIM_BULLSEYE: "矩形检测（瞄准靶心）",
    CMD_CIRCLE_TRACKING: "圆追踪检测",
    CMD_IDLE: "待机模式",
}

# ----------------- 4. 定义数据包格式 -----------------
TARGET_LOCKED_HEADER = b'\x3C\x3B'      # 数据包头
TARGET_LOCKED_FOOTER = b'\x01\x01'      # 数据包尾
TARGET_LOCKED_FORMAT = '>H'             # 16位整数包含XY坐标

# ----------------- 5. 辅助函数 -----------------
def calculate_distance(point1, point2):
    """计算两点间的欧几里得距离，增加健壮性"""
    try:
        if point1 is None or point2 is None:
            return 0
        return math.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)
    except (TypeError, IndexError):
        return 0

def setup_camera(resolution=ACTIVE_RESOLUTION):
    """配置摄像头进入指定模式和分辨率，返回成功与否"""
    try:
        sensor.reset()
        sensor.set_pixformat(sensor.GRAYSCALE)
        sensor.set_framesize(resolution)
        sensor.set_auto_gain(False)
        sensor.set_auto_whitebal(False)
        sensor.skip_frames(time=300)
        sensor.set_auto_exposure(True)
        sensor.skip_frames(time=500)
        sensor.set_auto_exposure(False)
        print("摄像头设置完成")
        return True
    except Exception as e:
        print(f"!!! 摄像头设置错误: {e}")
        return False

def find_target_boundary_rect(img):
    """在图像中寻找黑色边框作为靶纸边界（V1.8 强化版）"""
    try:
        blobs = img.find_blobs([BLACK_THRESHOLD], pixels_threshold=BINARY_PIXELS_THRESHOLD,
                               area_threshold=BINARY_AREA_THRESHOLD, merge=True)
        if not blobs:
            return None

        candidates = []
        for blob in blobs:
            rect_x, rect_y, rect_w, rect_h = blob.rect()
            if rect_w == 0 or rect_h == 0: continue

            # [鲁棒性强化 V1.8] 增加边缘贴合检查
            if rect_x == 0 or rect_y == 0 or \
               (rect_x + rect_w) >= SCREEN_WIDTH or \
               (rect_y + rect_h) >= SCREEN_HEIGHT:
                continue # 矩形贴边，跳过此候选

            rect_area = rect_w * rect_h
            blob_pixels = blob.area()

            # [鲁棒性强化 V1.6] 增加密度检查
            density = blob_pixels / rect_area if rect_area > 0 else 0
            is_very_large = rect_area > (SCREEN_WIDTH * SCREEN_HEIGHT * 0.75)
            if is_very_large and density > 0.6:
                continue

            aspect_ratio = max(rect_w, rect_h) / min(rect_w, rect_h)
            perimeter = blob.perimeter()
            shape_factor = (perimeter * perimeter) / (4 * math.pi * blob_pixels) if blob_pixels > 0 else 0

            if aspect_ratio < 4 and rect_area > 1000 and rect_area < (SCREEN_WIDTH * SCREEN_HEIGHT * 0.95) and shape_factor > 1.1:
                candidates.append((blob, rect_area))

        if not candidates:
            return None

        return max(candidates, key=lambda x: x[1])[0]
    except Exception as e:
        print(f"寻找边界错误: {e}")
        return None

def get_approximate_corners(boundary_rect):
    """获取矩形的近似四个角点"""
    try:
        if boundary_rect is None: return None
        x, y, w, h = boundary_rect.rect()
        return [(x, y), (x+w, y), (x+w, y+h), (x, y+h)]
    except Exception as e:
        print(f"获取角点错误: {e}")
        return None

def calculate_perspective_correction(corners):
    """根据四个角点计算透视校正，返回校正后的中心点"""
    if corners is None or len(corners) != 4:
        return SCREEN_CENTER_X, SCREEN_CENTER_Y

    try:
        x_sum = sum(corner[0] for corner in corners)
        y_sum = sum(corner[1] for corner in corners)
        simple_center_x, simple_center_y = x_sum // 4, y_sum // 4

        x1, y1 = corners[0]; x2, y2 = corners[2]
        x3, y3 = corners[1]; x4, y4 = corners[3]

        d = (x1-x2)*(y3-y4) - (y1-y2)*(x3-x4)
        if d == 0: return simple_center_x, simple_center_y

        px = ((x1*y2-y1*x2)*(x3-x4) - (x1-x2)*(x3*y4-y3*x4)) / d
        py = ((x1*y2-y1*x2)*(y3-y4) - (y1-y2)*(x3*y4-y3*x4)) / d
        return int(px), int(py)
    except Exception as e:
        print(f"透视校正计算错误: {e}")
        return simple_center_x, simple_center_y

def calculate_circle_params(rect_width, rect_height):
    """计算圆的参数，使用短边作为参考"""
    try:
        if rect_width is None or rect_height is None or rect_width <= 0 or rect_height <= 0:
            return 30, 20
        # 公式: r_6cm = (rect_height * 0.5782) / 2
        r_6cm = max(3, int(rect_height * 0.2891))
        return 0, r_6cm # r_10cm is not used, return 0
    except Exception as e:
        print(f"圆参数计算错误: {e}")
        return 30, 20

def calculate_ellipse_params(corners, circle_radius):
    """根据矩形四角点和基础圆半径计算椭圆参数"""
    if corners is None or len(corners) != 4 or circle_radius is None or circle_radius <= 0:
        return circle_radius, circle_radius, 0
    try:
        top_edge = calculate_distance(corners[0], corners[1])
        bottom_edge = calculate_distance(corners[3], corners[2])
        left_edge = calculate_distance(corners[0], corners[3])
        right_edge = calculate_distance(corners[1], corners[2])
        h_max = max(top_edge, bottom_edge)
        v_max = max(left_edge, right_edge)
        h_ratio = min(top_edge, bottom_edge) / h_max if h_max > 0 else 1.0
        v_ratio = min(left_edge, right_edge) / v_max if v_max > 0 else 1.0
        h_ellipse_ratio = 1 + (1 - max(0.1, h_ratio)) * 1.2
        v_ellipse_ratio = 1 + (1 - max(0.1, v_ratio)) * 1.2
        if h_ratio < v_ratio:
            a, b, rotation = circle_radius, circle_radius / h_ellipse_ratio, math.pi/2
        else:
            a, b, rotation = circle_radius, circle_radius / v_ellipse_ratio, 0
        return max(1, int(a)), max(1, int(b)), rotation
    except Exception as e:
        print(f"椭圆参数计算错误: {e}")
        return circle_radius, circle_radius, 0

def get_ellipse_point(center_x, center_y, a, b, angle, rotation=0):
    """计算椭圆上指定角度的点坐标"""
    try:
        if None in [center_x, center_y, a, b]: return SCREEN_CENTER_X, SCREEN_CENTER_Y
        a, b = max(1, int(a)), max(1, int(b))
        x = a * math.cos(angle)
        y = b * math.sin(angle)
        rotated_x = x * math.cos(rotation) - y * math.sin(rotation)
        rotated_y = x * math.sin(rotation) + y * math.cos(rotation)
        return int(center_x + rotated_x), int(center_y + rotated_y)
    except Exception as e:
        print(f"椭圆点计算错误: {e}")
        return int(center_x), int(center_y)

def send_target_coords(cx, cy, mode=None, center=None, radius=None, ellipse_params=None):
    """
    发送坐标到单片机。
    成功发送则返回一个包含(x, y)坐标的元组。
    如果因时间间隔未到而未发送，则返回(None, None)。
    """
    global last_uart_send_time, current_angle, circle_start_time, last_target_point
    try:
        current_time = time.ticks_ms()
        if time.ticks_diff(current_time, last_uart_send_time) < current_send_interval_ms:
            return None, None

        if mode == "circle" and center is not None and radius is not None:
            center_x, center_y = center
            if circle_start_time is None:
                circle_start_time = time.ticks_ms()
                current_angle = 0
            elapsed_time = time.ticks_diff(time.ticks_ms(), circle_start_time)
            ideal_angle = (elapsed_time % target_circle_time) / target_circle_time * 2 * math.pi
            angle_diff = ideal_angle - current_angle
            if angle_diff > math.pi: angle_diff -= 2 * math.pi
            elif angle_diff < -math.pi: angle_diff += 2 * math.pi
            current_angle = (current_angle + angle_diff * 0.3) % (2 * math.pi)
            if ellipse_params is None or None in ellipse_params:
                ellipse_params = (radius, radius, 0)
            a, b, rotation = ellipse_params
            target_x, target_y = get_ellipse_point(center_x, center_y, a, b, current_angle, rotation)
            if last_target_point:
                target_x = int(target_x * (1-smooth_factor) + last_target_point[0] * smooth_factor)
                target_y = int(target_y * (1-smooth_factor) + last_target_point[1] * smooth_factor)
            last_target_point = (target_x, target_y)
            final_x, final_y = min(255, max(0, target_x)), min(255, max(0, target_y))
        else:
            if cx is None or cy is None: cx, cy = SCREEN_CENTER_X, SCREEN_CENTER_Y
            final_x, final_y = min(255, max(0, int(cx))), min(255, max(0, int(cy)))
            circle_start_time = None
            last_target_point = None

        combined_coord = (final_x << 8) | final_y
        payload = struct.pack(TARGET_LOCKED_FORMAT, combined_coord)
        uart.write(TARGET_LOCKED_HEADER + payload + TARGET_LOCKED_FOOTER)
        last_uart_send_time = current_time
        return final_x, final_y
    except Exception as e:
        print(f"发送坐标错误: {e}")
        return None, None

def set_send_interval_for_mode(mode):
    """根据不同的工作模式设置相应的发送间隔"""
    global current_send_interval_ms
    if mode == STATE_CIRCLE_TRACKING:
        current_send_interval_ms = CIRCLE_SEND_INTERVAL_MS
    else:
        current_send_interval_ms = STANDARD_SEND_INTERVAL_MS
    print(f"  > [系统] 设置模式延迟: {current_send_interval_ms}ms")


def clear_uart_buffer():
    """清空串口接收缓冲区，增加超时保护"""
    clear_start_time = time.ticks_ms()
    while uart.any():
        uart.read()
        if time.ticks_diff(time.ticks_ms(), clear_start_time) > 200:
            print("!!! 清空缓冲区超时 !!!")
            break

# ==================================================================================
# === 主程序 ===
# ==================================================================================
if __name__ == "__main__":
    current_state = STATE_IDLE
    print_interval = 5 # 增加打印间隔，减少串口负载
    frame_count = 0

    print("\n" + "="*50)
    print(f"=== OpenMV 自行瞄准装置视觉控制程序 V1.9 ===")
    print(f"    分辨率: {SCREEN_WIDTH}x{SCREEN_HEIGHT} | 作者: Viny")
    print("="*50)
    print("系统已启动，正在初始化摄像头...")

    camera_ok = setup_camera()
    if not camera_ok:
        print("!!! 摄像头初始化失败，请检查连接后重启设备 !!!")
        while True: time.sleep_ms(1000)

    print("\n--- 可用串口指令 ---")
    for cmd, desc in COMMAND_MAP.items():
        print(f"  指令 {cmd.hex().upper()}: {desc}")
    print("="*50 + "\n当前处于待机状态，等待指令...")

    clear_uart_buffer()

    while True:
        try:
            # [鲁棒性] 检查摄像头状态
            if not camera_ok:
                print("检测到摄像头错误，尝试在1秒后重新初始化...")
                time.sleep_ms(1000)
                camera_ok = setup_camera()
                continue

            # ----------------- 串口通信处理优先 -----------------
            if uart.any():
                command = uart.read(1)
                clear_uart_buffer()

                if command in COMMAND_MAP:
                    cmd_desc = COMMAND_MAP[command]
                    print(f"\n>>> 收到指令: {command.hex().upper()} ({cmd_desc}) <<<")

                    if command == CMD_AIM_BULLSEYE:
                        current_state = STATE_AIM_BULLSEYE
                        set_send_interval_for_mode(STATE_AIM_BULLSEYE)
                    elif command == CMD_CIRCLE_TRACKING:
                        current_state = STATE_CIRCLE_TRACKING
                        set_send_interval_for_mode(STATE_CIRCLE_TRACKING)
                        circle_start_time = None
                        current_angle = 0
                        last_target_point = None
                    elif command == CMD_IDLE:
                        current_state = STATE_IDLE
                        print("返回待机模式")

                    time.sleep_ms(1)
                    continue

            # ----------------- 状态机执行 -----------------
            if current_state == STATE_IDLE:
                time.sleep_ms(50)
                continue

            frame_count += 1
            should_print = (frame_count % print_interval == 0)

            img = sensor.snapshot()
            boundary_rect = find_target_boundary_rect(img)
            source_method = "Blob"

            if not boundary_rect:
                if should_print: print(f"  > [{COMMAND_MAP.get(CMD_AIM_BULLSEYE if current_state == STATE_AIM_BULLSEYE else CMD_CIRCLE_TRACKING, '未知')}] 未找到目标")
            else:
                corners = get_approximate_corners(boundary_rect)
                if corners:
                    corrected_cx, corrected_cy = calculate_perspective_correction(corners)
                    rect_w, rect_h = boundary_rect.rect()[2], boundary_rect.rect()[3]

                    # --- [逻辑修正 V1.9] 核心修改点 ---
                    # 无论是否发送UART，只要找到目标就进行绘制，提供即时视觉反馈。

                    if current_state == STATE_AIM_BULLSEYE:
                        # 1. 总是绘制视觉反馈
                        img.draw_rectangle(boundary_rect.rect(), color=255, thickness=2)
                        img.draw_cross(corrected_cx, corrected_cy, color=255, size=3)

                        # 2. 尝试发送坐标（受时间门控）
                        result = send_target_coords(corrected_cx, corrected_cy)

                        # 3. 如果发送成功，则打印信息
                        if result and result[0] is not None:
                            if should_print:
                                print(f"  > [瞄准] 成功! 方法:{source_method}. 发送:({result[0]},{result[1]})")

                    elif current_state == STATE_CIRCLE_TRACKING:
                        # 1. 计算参数
                        _, r_6cm = calculate_circle_params(rect_w, rect_h)
                        ellipse_params = calculate_ellipse_params(corners, r_6cm)

                        # 2. 总是绘制静态视觉反馈
                        img.draw_rectangle(boundary_rect.rect(), color=255, thickness=2)
                        img.draw_cross(corrected_cx, corrected_cy, color=255, size=3)
                        img.draw_circle(corrected_cx, corrected_cy, r_6cm, color=255, thickness=1)

                        # 3. 尝试发送坐标（受时间门控）
                        result = send_target_coords(0, 0, mode="circle", center=(corrected_cx, corrected_cy),
                                                    radius=r_6cm, ellipse_params=ellipse_params)

                        # 4. 如果发送成功，绘制动态点并打印信息
                        if result and result[0] is not None:
                            target_x, target_y = result
                            img.draw_cross(target_x, target_y, color=128, size=3)
                            if should_print:
                                angle_deg = int(current_angle * 180 / math.pi)
                                print(f"  > [追踪] R:{r_6cm} | 点:({target_x},{target_y}) @{angle_deg}°")

            gc.collect()
            time.sleep_ms(1)

        except Exception as e:
            print(f"!!! 主循环发生严重错误: {type(e).__name__}: {e} !!!")
            if "sensor" in str(e).lower() or "snapshot" in str(e).lower():
                camera_ok = False
                print("错误可能与摄像头有关，将尝试在下一周期恢复。")
            time.sleep_ms(500)
