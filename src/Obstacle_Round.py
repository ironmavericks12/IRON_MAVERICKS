import cv2
import numpy as np
import RPi.GPIO as GPIO
from picamera2 import Picamera2
from time import sleep
import time

CLOCKWISE = None
LINE_COOLDOWN = 1.9
blue_detected = False
orange_detected = False
line_count = 0

# --- LAP / LINE CONFIG -----------------------------------------
# LINES_PER_LAP = how many gate detections happen per single lap.
# On most single-gate tracks this is 1 (robot only passes the
# blue/orange marker once per lap). If your track has multiple
# color markers per lap, change LINES_PER_LAP accordingly.
LAPS_TO_COMPLETE = 3
LINES_PER_LAP = 4   # robot crosses the same line 4x before 1 lap counts as done
total_lines = LAPS_TO_COMPLETE * LINES_PER_LAP
# -----------------------------------------------------------------

last_orange_time = 0.0
last_blue_time = 0.0
last_line_time = 0.0
prev_marker_seen = False   # tracks whether the marker was visible last frame

t_area = 600
KP = 0.035
f_speed = 60# 70 (kp = 0.05) 

CENTER = 95 # 98
RIGHT = 30 # 70
LEFT = 150 # 130

# ---------------- Obstacle Avoidance Config ----------------
# RED  -> pass on the RIGHT side of the obstacle  (steer toward RIGHT angle)
# GREEN -> pass on the LEFT side of the obstacle  (steer toward LEFT angle)
#
# "Near" = the obstacle's contour area is big enough that we should react.
# Tune OBSTACLE_MIN_AREA (detect at all) and OBSTACLE_NEAR_AREA (react/avoid)
# by watching the printed areas / debug rectangles and adjusting.
OBSTACLE_MIN_AREA = 400      # minimum area to even consider it a real obstacle
OBSTACLE_NEAR_AREA = 3500    # area threshold at which we start avoiding it
OBSTACLE_AVOID_OFFSET = 35   # bias added ON TOP of the normal line-follow angle
OBSTACLE_AVOID_SPEED = 40    # slow down a bit while avoiding, for safety

# Sign of the bias applied per color. Your servo's angle->turn direction
# turned out to be the opposite of the RIGHT/LEFT variable names, so this
# is flipped from the "obvious" +/-. If it's ever backwards again, just
# flip these two signs (don't touch anything else).
RED_AVOID_SIGN = +1     # RED  -> pass on the right -> bias sign that turns right on YOUR robot
GREEN_AVOID_SIGN = -1   # GREEN -> pass on the left  -> bias sign that turns left on YOUR robot
# --------------------------------------------------------------

# ---------------- Motor Pins ----------------
IN1 = 26
IN2 = 19
PWM_PIN = 13
SERVO_PIN = 12

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(PWM_PIN, GPIO.OUT)
GPIO.setup(SERVO_PIN, GPIO.OUT)

motor_pwm = GPIO.PWM(PWM_PIN, 1000)
motor_pwm.start(0)

servo_pwm = GPIO.PWM(SERVO_PIN, 50)
servo_pwm.start(0)



last_angle = -1

current_angle = CENTER
last_servo_time = 0

def steer(angle):

    angle = max(RIGHT, min(LEFT, angle))      # Limit angle

    duty = 2.5 + (angle / 180.0) * 10.0  # Convert angle to duty cycle

    servo_pwm.ChangeDutyCycle(duty)

    sleep(0.01)

    servo_pwm.ChangeDutyCycle(0)    
def forward(speed):
    # If your motor moves backwards,
    # change HIGH to LOW
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    motor_pwm.ChangeDutyCycle(speed)

def stop():
    motor_pwm.ChangeDutyCycle(0)
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    
def compute_line_angle(is_clockwise):
    """
    Same math your clockwise()/anticlockwise() used, just returned as a
    number instead of steering immediately. Lets obstacle avoidance add
    a bias on top of this instead of replacing it outright.
    """
    if left_target and right_target:

        left_x, left_y = left_target
        right_x, right_y = right_target

        left_distance = left_x
        right_distance = WIDTH - right_x

        error = left_distance - right_distance

        return CENTER + error * KP

    elif left_target:

        only_x, _ = left_target
        return CENTER + ((only_x - 200) * KP)

    elif right_target:

        only_x, _ = right_target
        if is_clockwise:
            return CENTER + ((only_x + (WIDTH - 200)) * KP)
        else:
            return CENTER + ((only_x - (WIDTH - 200)) * KP)

    else:
        return CENTER


def clockwise():
    steer(compute_line_angle(True))


def anticlockwise():
    steer(compute_line_angle(False))


    
    

# ============================================================
# START
# ============================================================

# ==========================================================
# CAMERA SETTINGS
# ==========================================================
WIDTH = 1080
HEIGHT = 680
X_MID = WIDTH // 2

# Ignore the top N rows of the frame entirely - nothing detected there
# (cuts out background clutter/shelving above the black wall).
# Tune this: hover your mouse on the debug window right where the
# black wall ends and the white floor starts, read the y value shown
# in the coordinate readout, and set it here.
ROI_TOP_IGNORE = 120

# Ignore the leftmost/rightmost EDGE_IGNORE_PERCENT of columns for
# BLUE/ORANGE marker detection only. Anything in those side margins
# is only allowed to be seen as black wall - blue/orange found there
# is zeroed out before contour detection, so it can never register as
# a marker crossing. The black mask itself stays full-width.
EDGE_IGNORE_PERCENT = 10
EDGE_IGNORE_PX = int(WIDTH * EDGE_IGNORE_PERCENT / 100)

# Black track line stays LAB-only (unchanged).
BLACK_LOWER = np.array([0,118,118])
BLACK_UPPER = np.array([75,138,138])

# ---- HSV ranges ---------------------------------------------------
LOWER_GREEN = np.array([35, 70, 50]);    UPPER_GREEN = np.array([85, 255, 255])
LOWER_RED1 = np.array([0, 120, 80]);     UPPER_RED1 = np.array([10, 255, 255])
LOWER_RED2 = np.array([170, 120, 80]);   UPPER_RED2 = np.array([180, 255, 255])
LOWER_MAGENTA = np.array([135, 70, 70]); UPPER_MAGENTA = np.array([170, 255, 255])
LOWER_ORANGE = np.array([8, 130, 100]);  UPPER_ORANGE = np.array([22, 255, 255])
LOWER_BLUE = np.array([95, 100, 60]);    UPPER_BLUE = np.array([130, 255, 255])
# ---- LAB channel gating (A = green-red axis, B = blue-yellow axis) ----
RED_A_MIN = 155          # red -> high A
GREEN_A_MAX = 110        # green -> low A
MAGENTA_A_MIN = 150      # magenta -> high A
ORANGE_B_MIN = 145       # orange -> high B (yellow side)
BLUE_B_MAX = 115         # blue -> low B (blue side)
# NOTE: magenta is defined for future use (e.g. an extra marker color)
# but nothing in the current logic reacts to it yet.
# ---------------------------------------------------------------------

picam2 = Picamera2()

config = picam2.create_preview_configuration(
    main={"size": (WIDTH, HEIGHT), "format": "RGB888"}
)

picam2.configure(config)

picam2.start()

print("Auto adjusting camera...")
picam2.set_controls({
    "AeEnable": True,
    "AwbEnable": True
})

time.sleep(2)

meta = picam2.capture_metadata()

exp = meta["ExposureTime"]
gain = meta["AnalogueGain"]

picam2.set_controls({
    "AeEnable": False,
    "AwbEnable": False,
    "ExposureTime": exp,
    "AnalogueGain": gain
})

print("Camera locked")
print("Exposure:", exp)
print("Gain:", gain)

clahe = cv2.createCLAHE(clipLimit=2.5, tileGridSize=(8,8))
kernel = np.ones((5,5), np.uint8)

fps_time = time.time()

steer(CENTER)
sleep(0.7)
forward(f_speed) # 60s = 0.02kp  50s = 0.012kP

print(f"Robot Started - will stop after {LAPS_TO_COMPLETE} laps ({total_lines} gate crossings)")


def hsv_lab_mask(hsv, hsv_lower, hsv_upper, lab_channel, lab_min, lab_max):
    """
    Build a mask from an HSV color range AND-gated by a LAB channel range.
    The HSV range gets the rough color family; the LAB gate (A or B channel)
    tightens it against the axis that color sits on, cutting false positives
    from lighting changes / similar hues.
    """
    hsv_mask = cv2.inRange(hsv, hsv_lower, hsv_upper)
    lab_gate = cv2.inRange(lab_channel, lab_min, lab_max)
    return cv2.bitwise_and(hsv_mask, lab_gate)


def get_largest_obstacle(mask, min_area):
    """
    Returns (detected, area, bbox) for the largest contour in mask
    that clears min_area. bbox = (x, y, w, h). detected=False if none.
    """
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return False, 0, None

    largest = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(largest)

    if area < min_area:
        return False, 0, None

    bbox = cv2.boundingRect(largest)
    return True, area, bbox


while True:

# Turn PWM off after 30 ms
    if time.time() - last_servo_time > 0.03:
        servo_pwm.ChangeDutyCycle(0)

    frame = picam2.capture_array()
    #frame = cv2.flip(frame, 1)  # camera is rear-facing - mirror so left/right logic matches robot's actual direction
    frame = cv2.GaussianBlur(frame, (5,5), 0)

    lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    l,a,b = cv2.split(lab)
    l = clahe.apply(l)
    lab = cv2.merge((l,a,b))
    # NOTE: a/b are the RAW (pre-CLAHE) chroma channels - CLAHE above only
    # touches lightness (l), so a/b are exactly what we want to gate on.

    black_mask = cv2.inRange(lab, BLACK_LOWER, BLACK_UPPER)
    black_mask[:ROI_TOP_IGNORE, :] = 0

    # ---- Build orange mask early (HSV + LAB-B gate) so we can exclude ----
    # its pixels from the black mask, same as before.
    orange_raw = hsv_lab_mask(hsv, LOWER_ORANGE, UPPER_ORANGE, b, ORANGE_B_MIN, 255)
    orange_raw[:ROI_TOP_IGNORE, :] = 0
    orange_exclude = cv2.morphologyEx(orange_raw, cv2.MORPH_CLOSE, np.ones((5,5), np.uint8))
    black_mask = cv2.bitwise_and(black_mask, cv2.bitwise_not(orange_exclude))

    black_mask = cv2.morphologyEx(black_mask, cv2.MORPH_OPEN, kernel)
    black_mask = cv2.morphologyEx(black_mask, cv2.MORPH_CLOSE, kernel)
    black_mask = cv2.dilate(black_mask, kernel, iterations=1)
       
    black_contours, _ = cv2.findContours(black_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    output = frame.copy()
    cv2.line(output, (0, ROI_TOP_IGNORE), (WIDTH, ROI_TOP_IGNORE), (0, 255, 255), 2)
    # visualize the marker-ignore side margins
    cv2.line(output, (EDGE_IGNORE_PX, 0), (EDGE_IGNORE_PX, HEIGHT), (0, 255, 0), 1)
    cv2.line(output, (WIDTH - EDGE_IGNORE_PX, 0), (WIDTH - EDGE_IGNORE_PX, HEIGHT), (0, 255, 0), 1)
    
    left_target = None
    right_target = None

    left_bottom = -1
    right_bottom = -1

    for cnt in black_contours:
        area = cv2.contourArea(cnt)
        if area < t_area:
            continue
        x, y, w, h = cv2.boundingRect(cnt)
        cv2.rectangle(output, (x, y), (x+w, y+h), (255,255,0), 2)
        cx = x + w//2
        bottom = y + h
        if cx < X_MID:
            if bottom > left_bottom:
                left_bottom = bottom
                left_target = (x+w, bottom)
                cv2.circle(output, left_target, 8, (255,0,0), -1)
        else:
            if bottom > right_bottom:
                right_bottom = bottom
                right_target = (x, bottom)
                cv2.circle(output, right_target, 8, (0,0,255), -1)

    # ---------------- BLUE / ORANGE LAP MARKERS (HSV + LAB gated) ----------------
    blue_detected = False
    blue_mask = hsv_lab_mask(hsv, LOWER_BLUE, UPPER_BLUE, b, 0, BLUE_B_MAX)
    blue_mask[:ROI_TOP_IGNORE, :] = 0
    # zero out left/right edge margins - blue in those columns is not a marker
    blue_mask[:, :EDGE_IGNORE_PX] = 0
    blue_mask[:, WIDTH-EDGE_IGNORE_PX:] = 0
    blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_OPEN, kernel)
    blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_CLOSE, kernel)
    blue_mask = cv2.dilate(blue_mask, kernel, iterations=1)
    blue_contours, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if blue_contours:
        largest = max(blue_contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)
        if area > 800:
            x, y, w, h = cv2.boundingRect(largest)
            if (x+w) > 300:
                blue_detected = True
            cv2.rectangle(output, (x, y), (x+w, y+h), (255,0,0), 2)

    orange_detected = False
    orange_mask = orange_raw.copy()
    # zero out left/right edge margins - orange in those columns is not a marker
    orange_mask[:, :EDGE_IGNORE_PX] = 0
    orange_mask[:, WIDTH-EDGE_IGNORE_PX:] = 0
    orange_mask = cv2.morphologyEx(orange_mask, cv2.MORPH_OPEN, kernel)
    orange_mask = cv2.morphologyEx(
        orange_mask,
        cv2.MORPH_CLOSE,
        np.ones((21,21), np.uint8)
    )
    orange_mask = cv2.dilate(
        orange_mask,
        np.ones((15,15), np.uint8),
        iterations=2
    )
    orange_contours, _ = cv2.findContours(orange_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    all_points = []

    for cnt in orange_contours:
        if cv2.contourArea(cnt) > 50:
            all_points.append(cnt)

    if all_points:
        merged = np.vstack(all_points)
        x, y, w, h = cv2.boundingRect(merged)
        total_area = sum(cv2.contourArea(c) for c in all_points)
        if total_area > 800:
            orange_detected = True
        cv2.rectangle(output, (x, y), (x+w, y+h), (0,165,255), 2)

    # ---------------- RED / GREEN OBSTACLE DETECTION (HSV + LAB gated) ----------------
    # Full-width (no edge-ignore) since obstacles can appear anywhere
    # on the track, not just near the lap-marker gate.
    red_raw = cv2.bitwise_or(
        cv2.inRange(hsv, LOWER_RED1, UPPER_RED1),
        cv2.inRange(hsv, LOWER_RED2, UPPER_RED2)
    )
    red_lab_gate = cv2.inRange(a, RED_A_MIN, 255)
    red_mask = cv2.bitwise_and(red_raw, red_lab_gate)
    red_mask[:ROI_TOP_IGNORE, :] = 0
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel)
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel)
    red_mask = cv2.dilate(red_mask, kernel, iterations=1)

    green_mask = hsv_lab_mask(hsv, LOWER_GREEN, UPPER_GREEN, a, 0, GREEN_A_MAX)
    green_mask[:ROI_TOP_IGNORE, :] = 0
    green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_OPEN, kernel)
    green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_CLOSE, kernel)
    green_mask = cv2.dilate(green_mask, kernel, iterations=1)

    red_found, red_area, red_bbox = get_largest_obstacle(red_mask, OBSTACLE_MIN_AREA)
    green_found, green_area, green_bbox = get_largest_obstacle(green_mask, OBSTACLE_MIN_AREA)

    if red_found:
        x, y, w, h = red_bbox
        cv2.rectangle(output, (x, y), (x+w, y+h), (0, 0, 255), 2)
        cv2.putText(output, f"RED {int(red_area)}", (x, max(0, y-8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

    if green_found:
        x, y, w, h = green_bbox
        cv2.rectangle(output, (x, y), (x+w, y+h), (0, 255, 0), 2)
        cv2.putText(output, f"GREEN {int(green_area)}", (x, max(0, y-8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    # Decide which obstacle is nearer. Bigger contour area = closer to
    # the camera, since the camera looks forward/down at the track.
    # Only the NEARER one is reacted to; if it's not "near" enough yet
    # (area < OBSTACLE_NEAR_AREA) we keep following the line normally.
    avoid_color = None
    if red_found and green_found:
        avoid_color = "RED" if red_area >= green_area else "GREEN"
    elif red_found:
        avoid_color = "RED"
    elif green_found:
        avoid_color = "GREEN"

    nearest_area = red_area if avoid_color == "RED" else green_area if avoid_color == "GREEN" else 0
    obstacle_avoiding = avoid_color is not None and nearest_area >= OBSTACLE_NEAR_AREA
    # -------------------------------------------------------------------
    
    current_time = time.time()

    if CLOCKWISE is None:
        if blue_detected:
            CLOCKWISE = False
            last_line_time = current_time   # start cooldown, don't count yet
            print("ANTICLOCKWISE")

        elif orange_detected:
            CLOCKWISE = True
            last_line_time = current_time   # start cooldown, don't count yet
            print("CLOCKWISE")

    else:
        # Count crossings of the SAME marker color that set the direction.
        # Only count on the RISING EDGE (marker just appeared) so a single
        # slow crossing, or the line staying in view for several frames,
        # doesn't get counted multiple times.
        marker_seen = orange_detected if CLOCKWISE else blue_detected
        if marker_seen and not prev_marker_seen and current_time - last_line_time > LINE_COOLDOWN:
            line_count += 1
            last_line_time = current_time
            #print("Line :", line_count)
        prev_marker_seen = marker_seen
    if line_count >= total_lines:
        steer(CENTER)
        forward(20)
        sleep(0.5)
        stop()
        print(f"{LAPS_TO_COMPLETE} laps complete - stopping")
        break
    print(f"Line {line_count} / {total_lines}  (lap {line_count // LINES_PER_LAP} of {LAPS_TO_COMPLETE})")

    if obstacle_avoiding:
        # Start from the NORMAL line-following angle (so we still respect
        # the black wall / track edges) and bias it toward the correct
        # side of the obstacle, instead of blindly overriding steering.
        base_angle = compute_line_angle(CLOCKWISE == True) if CLOCKWISE is not None else CENTER

        if avoid_color == "RED":
            avoid_angle = base_angle + (RED_AVOID_SIGN * OBSTACLE_AVOID_OFFSET)
        else:
            avoid_angle = base_angle + (GREEN_AVOID_SIGN * OBSTACLE_AVOID_OFFSET)

        steer(avoid_angle)
        forward(OBSTACLE_AVOID_SPEED)
        print(f"AVOIDING {avoid_color} obstacle (area={int(nearest_area)}) base={base_angle:.1f} -> angle {avoid_angle:.1f}")
    else:
        # No near obstacle - resume normal speed/line-following
        forward(f_speed)
        if CLOCKWISE == True:
            clockwise()
        else:
            anticlockwise()
    
        
        
    cv2.imshow("Original", output)
    #cv2.imshow("Mask", mask)

    key = cv2.waitKey(1) & 0xFF

    if key == ord('q'):
        steer(CENTER)
        stop()
        #video.release()
        break

cv2.destroyAllWindows()
picam2.stop()