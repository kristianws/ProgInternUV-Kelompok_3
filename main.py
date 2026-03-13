import math
import cv2
from ultralytics import YOLO
import collections
try:
    from collections import abc
    collections.MutableMapping = abc.MutableMapping
except:
    pass

from dronekit import connect, VehicleMode, Command
from pymavlink import mavutil
import time

print('Connecting...')
vehicle = connect('udp:127.0.0.1:14551', wait_ready=True)
# vehicle = connect('/dev/ttyACM0')

def arm_and_takeoff(altitude):
    while not vehicle.is_armable:
        print("waiting to be armable")
        # time.sleep(1)
    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    print("Taking Off")
    vehicle.simple_takeoff(altitude)

    while True:
        v_alt = vehicle.location.global_relative_frame.alt
        if v_alt >= altitude - 1.0:
            print("Target altitude reached")
            break
          


def get_zone(cx, left_boundary, right_boundary):
    if cx < left_boundary:
        return "LEFT"
    elif cx > right_boundary:
        return "RIGHT"
    else:
        return "CENTER"
      

def draw_grid(frame, left_boundary, right_boundary, frame_h):
    cv2.line(frame, (left_boundary, 0), (left_boundary, frame_h), (0, 255, 255), 2)
    cv2.line(frame, (right_boundary, 0), (right_boundary, frame_h), (0, 255, 255), 2)

    label_y = frame_h - 15
    cv2.putText(frame, "LEFT",   (10, label_y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)
    cv2.putText(frame, "CENTER", (left_boundary + 5, label_y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)
    cv2.putText(frame, "RIGHT",  (right_boundary + 5, label_y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)


def draw_hud(frame, fps, nav_status, yaw_deg, frame_h):
    cv2.putText(frame, f"FPS    : {fps:.1f}",         (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    cv2.putText(frame, f"Status : {nav_status}",       (10, 50),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    cv2.putText(frame, f"Yaw    : {yaw_deg:.1f} deg/s", (10, 75),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)


# fungsi untuk bergerak dengan kecepatan tertentu dan yaw rate tertentu
def gerak(vx,yaw_rate):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,
        0, 0,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        0b0000011111000111,
        0, 0, 0,
        vx, 0, 0,
        0, 0, 0,
        0, yaw_rate
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()

model = YOLO('/home/risanws/learning/kuliah/bengawan/final-project/ball-detection-yolov5.pt')
model.conf = 0.9 
model = YOLO('/home/risanws/learning/kuliah/bengawan/final-project/yolov8s.pt')
cap = cv2.VideoCapture(0)
    

def ball_zone(ball):
    x1, y1, x2, y2 = ball[0], ball[1], ball[2], ball[3]
    cx = (x1 + x2) / 2
    return get_zone(cx, left_boundary, right_boundary)


model = torch.hub.load(
    'ultralytics/yolov5', 
    'custom', 
    path='./ball-detection-yolov5.pt', 
    source='local'
)

model.conf = 0.9  # confidence threshold

cap = cv2.VideoCapture(1)
model = YOLO('/home/risanws/learning/kuliah/bengawan/final-project/ball-detection-yolov5.pt')
model.conf = 0.9 
cap = cv2.VideoCapture(0)
arm_and_takeoff(0)

prev_time = time.time()  # Waktu sebelumnya untuk menghitung FPS
ground_speed = 2

CONF_THRESHOLD    = 0.8          # ambang batas confidence
CONF_THRESHOLD    = 0.6          # ambang batas confidence
CONF_THRESHOLD    = 0.9          # ambang batas confidence
CONF_THRESHOLD    = 0.8          # ambang batas confidence
CENTER_HALF_WIDTH = 75           # lebar setengah zona tengah (piksel)
LARGE_BOX_RATIO   = 0.60         # bounding box dianggap "dekat" jika > 70% luas frame
MAX_YAW_NORMAL    = math.radians(30)   # yaw rate normal (belok)
MAX_YAW_CLOSE     = math.radians(40)   # yaw rate saat objek terlalu dekat
YAW_STEP          = math.radians(3)    # kecepatan perubahan yaw per frame (smoothing)
CENTER_HALF_WIDTH = 100
MAX_YAW_NORMAL    = math.radians(30)
MAX_YAW_CLOSE     = math.radians(40)
YAW_STEP          = math.radians(3)
GROUND_SPEED      = 2
BLACK_CLOSE_RATIO = 0.08   # 8% luas frame

current_yaw_rate = 0.0 

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break
      
    
    frame_h, frame_w = frame.shape[:2]
    frame_area = frame_w * frame_h
    
    center_x      = frame_w // 2
    left_boundary = center_x - CENTER_HALF_WIDTH
    right_boundary = center_x + CENTER_HALF_WIDTH
      
    results = model(frame, imgsz=416)
    results = model(frame, imgsz=512)
    results = model(frame, imgsz=416)
    boxes   = results[0].boxes
    
    high_conf = sorted(
        [b for b in boxes if float(b.conf[0]) > CONF_THRESHOLD],
        key=lambda b: float(b.conf[0]),
        reverse=True
    )[:2]
    
    red_ball   = None
    green_ball = None
    black_ball = None
    for box in high_conf:
        cls_name = model.names[int(box.cls[0])].lower()
        if any(k in cls_name for k in ('bola_merah')):
            red_ball = box
        elif any(k in cls_name for k in ('bola_hijau')):
            green_ball = box
        elif any(k in cls_name for k in ('bola_hitam')):
            black_ball = box
    for box in high_conf:
        cls_name = model.names[int(box.cls[0])].lower()
        if any(k in cls_name for k in ('bola_merah')):
            red_ball = box
        elif any(k in cls_name for k in ('bola_hijau')):
            green_ball = box
        elif any(k in cls_name for k in ('bola_hitam')):
            black_ball = box
    
    target_yaw_rate = 0.0
    nav_status      = "SEARCHING..."

    if red_ball is not None and green_ball is not None:
        rx1, ry1, rx2, ry2 = red_ball.xyxy[0].tolist()
        gx1, gy1, gx2, gy2 = green_ball.xyxy[0].tolist()
        bx1, by1, bx2, by2 = black_ball.xyxy[0].tolist()

        red_cx   = (rx1 + rx2) / 2
        green_cx = (gx1 + gx2) / 2
        black_cx = (bx1 + bx2) / 2

        red_zone   = get_zone(rx1,   left_boundary, right_boundary)
        green_zone = get_zone(gx1, left_boundary, right_boundary)
        black_zone = get_zone(bx1, left_boundary, right_boundary)

        if red_zone == "LEFT" and green_zone == "RIGHT":
            target_yaw_rate = 0.0
            nav_status      = "STRAIGHT ✓"

        elif red_zone == "RIGHT":
            target_yaw_rate = MAX_YAW_NORMAL
            nav_status      = "TURN LEFT ← (red left)"

        elif green_zone == "LEFT":
            target_yaw_rate = -MAX_YAW_NORMAL
            nav_status      = "TURN RIGHT → (green right)"
        
        elif black_zone == "CENTER":
            target_yaw_rate = MAX_YAW_NORMAL
            nav_status      = "TURN AROUND"

        red_cx   = (rx1 + rx2) / 2
        green_cx = (gx1 + gx2) / 2
        black_cx = (bx1 + bx2) / 2

        red_zone   = get_zone(red_cx,   left_boundary, right_boundary)
        green_zone = get_zone(green_cx, left_boundary, right_boundary)
        black_zone = get_zone(black_cx, left_boundary, right_boundary)

        # Luas bounding box relatif terhadap frame
        red_area_ratio   = ((rx2 - rx1) * (ry2 - ry1)) / frame_area
        green_area_ratio = ((gx2 - gx1) * (gy2 - gy1)) / frame_area
        black_area_ratio = ((bx2 - bx1) * (by2 - by1)) / frame_area

        red_is_large   = red_area_ratio   > LARGE_BOX_RATIO
        green_is_large = green_area_ratio > LARGE_BOX_RATIO
        black_is_large = black_area_ratio > LARGE_BOX_RATIO

        if red_zone == "LEFT" and green_zone == "RIGHT":
            target_yaw_rate = 0.0
            nav_status      = "STRAIGHT ✓"

        # elif (red_is_large or green_is_large) and (red_zone == "CENTER" or green_zone == "CENTER"):
        #     # Arah yaw mengikuti kondisi sebelumnya (pertahankan tanda)
        #     direction       = 1 if current_yaw_rate >= 0 else -1
        #     target_yaw_rate = direction * MAX_YAW_CLOSE
        #     nav_status      = "CLOSE! MAX YAW ⬆"

        # elif red_zone == "RIGHT":
        #     target_yaw_rate = -MAX_YAW_NORMAL
        #     nav_status      = "TURN LEFT ← (red left)"

        elif green_zone == "RIGHT":
            target_yaw_rate = MAX_YAW_NORMAL
            nav_status      = "TURN RIGHT → (green right)"
        
        elif black_zone == "CENTER":
            target_yaw_rate = MAX_YAW_NORMAL
            nav_status      = "TURN AROUND"
        # elif green_zone == "LEFT":
        #     target_yaw_rate = -MAX_YAW_NORMAL
        #     nav_status      = "TURN RIGHT → (green right)"
        
        # elif black_zone == "CENTER":
        #     target_yaw_rate = MAX_YAW_NORMAL
        #     nav_status      = "TURN AROUND"

current_yaw_rate = 0.0

if __name__ == "__main__":
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break
          
        # resize frame
        frame                = cv2.resize(frame, (512, 512))
        frame_h, frame_w     = frame.shape[:2]
        frame_area           = frame_h * frame_w
        center_x             = frame_w // 2
        left_boundary        = center_x - CENTER_HALF_WIDTH
        right_boundary       = center_x + CENTER_HALF_WIDTH
          
        results = model(frame, imgsz=512)
        detections = results.xyxy[0].cpu().numpy()
        
        red_candidates   = []
        green_candidates = []
        black_candidates = []

        for box in detections:
            cls_name = model.names[int(box.cls[0])].lower()
            if 'bola_merah' in cls_name:
                red_candidates.append(box)
            elif 'bola_hijau' in cls_name:
                green_candidates.append(box)
            elif 'bola_hitam' in cls_name:
                black_candidates.append(box)
        
        
        red_ball   = pick_closest(red_candidates,   frame) if red_candidates   else None
        green_ball = pick_closest(green_candidates, frame) if green_candidates else None

    elif red_ball is not None:
        rx1, _, rx2, _ = red_ball.xyxy[0].tolist()
        red_cx   = (rx1 + rx2) / 2
        red_zone = get_zone(red_cx, left_boundary, right_boundary)
        if red_zone == "RIGHT":
            target_yaw_rate = MAX_YAW_NORMAL
            nav_status      = "TURN RIGHT ← (only red)"
        # elif red_zone == "LEFT":
        #     target_yaw_rate = -MAX_YAW_NORMAL
        #     nav_status      = "TURN RIGHT → (only red)"
        if red_zone == "LEFT":
            target_yaw_rate = -MAX_YAW_NORMAL
            nav_status      = "TURN LEFT ← (only red)"
        if red_zone == "RIGHT":
            target_yaw_rate = MAX_YAW_NORMAL
            nav_status      = "TURN LEFT ← (only red)"
        # elif red_zone == "LEFT":
        #     target_yaw_rate = MAX_YAW_NORMAL
        #     nav_status      = "TURN RIGHT → (only red)"

    elif green_ball is not None:
        gx1, _, gx2, _ = green_ball.xyxy[0].tolist()
        green_cx   = (gx1 + gx2) / 2
        green_zone = get_zone(green_cx, left_boundary, right_boundary)
        if green_zone == "LEFT":
            target_yaw_rate = -MAX_YAW_NORMAL
            nav_status      = "TURN LEFT → (only green)"
      
    elif black_ball is not None:
      bx1, _, bx2, _ = black_ball.xyxy[0].tolist()
      black_cx   = (bx1 + bx2) / 2
      black_zone = get_zone(black_cx, left_boundary, right_boundary)
      if black_zone == "CENTER":
          target_yaw_rate = MAX_YAW_NORMAL
          nav_status      = "TURN AROUND (only black)"
    
    else:
        target_yaw_rate = 0.0
        # ground_speed = 0
        nav_status      = "SEARCHING..."
        
    # ─── Smooth yaw rate (lerp per-frame step) ─────────────────────────────────
    # diff = target_yaw_rate - current_yaw_rate
    # if abs(diff) <= YAW_STEP:
    #     current_yaw_rate = target_yaw_rate
    # else:
    #     current_yaw_rate += math.copysign(YAW_STEP, diff)

    gerak(ground_speed, target_yaw_rate)
        if green_zone == "RIGHT":
            target_yaw_rate = MAX_YAW_NORMAL
        if green_zone == "LEFT":
            target_yaw_rate = -MAX_YAW_NORMAL
            nav_status      = "TURN RIGHT → (only green)"
      
    elif black_ball is not None:
      bx1, _, bx2, _ = black_ball.xyxy[0].tolist()
      black_cx   = (bx1 + bx2) / 2
      black_zone = get_zone(black_cx, left_boundary, right_boundary)
      if black_zone == "CENTER":
          target_yaw_rate = MAX_YAW_NORMAL
          nav_status      = "TURN AROUND (only black)"
    
    else:
        # Bola hitam: harus dekat kamera (effective area > threshold)
        #             DAN berada di tengah frame
        black_ball = None
        if black_candidates:
            for b in black_candidates:
                eff_ratio      = get_effective_area(b, frame) / frame_area
                close_enough   = eff_ratio > BLACK_CLOSE_RATIO
                centered       = is_centered_in_frame(b, frame, margin=0.30)
                if close_enough and centered:
                    # Dari semua yang memenuhi syarat, pilih yang terdekat
                    if black_ball is None or get_effective_area(b, frame) > get_effective_area(black_ball, frame):
                        black_ball = b
                        
        
        target_yaw_rate = 0.0
        nav_status      = "SEARCHING..."

        if red_ball is not None and green_ball is not None:
            red_zone   = ball_zone(red_ball)
            green_zone = ball_zone(green_ball)

            if red_zone == "LEFT" and green_zone == "RIGHT":
                target_yaw_rate = 0.0
                nav_status      = "STRAIGHT ✓"

            elif red_zone in ("RIGHT", "CENTER"):
                target_yaw_rate = -MAX_YAW_NORMAL
                nav_status      = f"TURN LEFT ← (red={red_zone})"

            elif green_zone in ("LEFT", "CENTER"):
                target_yaw_rate = MAX_YAW_NORMAL
                nav_status      = f"TURN RIGHT → (green={green_zone})"

        elif red_ball is not None:
            red_zone = ball_zone(red_ball)

            if red_zone in ("RIGHT", "CENTER"):
                target_yaw_rate = -MAX_YAW_NORMAL
                nav_status      = f"TURN LEFT ← (only red={red_zone})"
            else:
                target_yaw_rate = 0.0
                nav_status      = "HOLD (red at LEFT)"

        elif green_ball is not None:
            green_zone = ball_zone(green_ball)

            if green_zone in ("LEFT", "CENTER"):
                target_yaw_rate = MAX_YAW_NORMAL
                nav_status      = f"TURN RIGHT → (only green={green_zone})"
            else:
                target_yaw_rate = 0.0
                nav_status      = "HOLD (green at RIGHT)"

        elif black_ball is not None:
            black_zone = ball_zone(black_ball)

            if black_zone == "CENTER":
                target_yaw_rate = MAX_YAW_CLOSE
                nav_status      = "AVOID! TURN AROUND (black CENTER+CLOSE)"
            else:
                target_yaw_rate = 0.0
                nav_status      = f"HOLD (black={black_zone}, not centered)"

        else:
            target_yaw_rate = 0.0
            nav_status      = "HOLD"

    elif red_ball is not None:
        rx1, _, rx2, _ = red_ball.xyxy[0].tolist()
        red_cx   = (rx1 + rx2) / 2
        red_zone = get_zone(red_cx, left_boundary, right_boundary)
        if red_zone == "RIGHT":
            target_yaw_rate = MAX_YAW_NORMAL
            nav_status      = "TURN RIGHT ← (only red)"
        # elif red_zone == "LEFT":
        #     target_yaw_rate = -MAX_YAW_NORMAL
        #     nav_status      = "TURN RIGHT → (only red)"

    elif green_ball is not None:
        gx1, _, gx2, _ = green_ball.xyxy[0].tolist()
        green_cx   = (gx1 + gx2) / 2
        green_zone = get_zone(green_cx, left_boundary, right_boundary)
        if green_zone == "LEFT":
            target_yaw_rate = -MAX_YAW_NORMAL
            nav_status      = "TURN LEFT → (only green)"
      
    elif black_ball is not None:
      bx1, _, bx2, _ = black_ball.xyxy[0].tolist()
      black_cx   = (bx1 + bx2) / 2
      black_zone = get_zone(black_cx, left_boundary, right_boundary)
      if black_zone == "CENTER":
          target_yaw_rate = MAX_YAW_NORMAL
          nav_status      = "TURN AROUND (only black)"
    
    else:
        target_yaw_rate = 0.0
        # ground_speed = 0
        nav_status      = "SEARCHING..."
        
    # ─── Smooth yaw rate (lerp per-frame step) ─────────────────────────────────
    # diff = target_yaw_rate - current_yaw_rate
    # if abs(diff) <= YAW_STEP:
    #     current_yaw_rate = target_yaw_rate
    # else:
    #     current_yaw_rate += math.copysign(YAW_STEP, diff)

    gerak(ground_speed, current_yaw_rate)
    gerak(ground_speed, target_yaw_rate)

    # ─── Visualisasi ───────────────────────────────────────────────────────────
    annoted = results[0].plot()

    # Gambar grid SEBELUM flip agar posisi label sesuai tampilan
    draw_grid(annoted, left_boundary, right_boundary, frame_h)


    curr_time = time.time()
    fps       = 1 / (curr_time - prev_time + 1e-9)
    prev_time = curr_time
    draw_hud(annoted, fps, nav_status, math.degrees(current_yaw_rate), frame_h)
    # annoted = cv2.flip(annoted, 1)  # Flip horizontal untuk tampilan seperti cermin

    annoted = cv2.resize(annoted, (416, 416))  # Resize untuk tampilan lebih kecil
    cv2.imshow("YOLOv8 Boat Navigation", annoted)
    print(nav_status)
    # annoted = cv2.flip(annoted, 1)  # Flip horizontal untuk tampilan seperti cermin

    cv2.imshow("YOLOv8 Boat Navigation", annoted)
    print(nav_status)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    
    
cap.release()
cv2.destroyAllWindows()
vehicle.close()

print("Mission Completed")

        else:
            target_yaw_rate = 0.0
            nav_status      = "SEARCHING..."
            
        diff = target_yaw_rate - current_yaw_rate
        if abs(diff) <= YAW_STEP:
            current_yaw_rate = target_yaw_rate
        else:
            current_yaw_rate += math.copysign(YAW_STEP, diff)

        gerak(GROUND_SPEED, current_yaw_rate)

        # ─── Visualisasi ───────────────────────────────────────────────────────────
        annoted = results[0].plot()

        # Gambar grid SEBELUM flip agar posisi label sesuai tampilan
        draw_grid(annoted, left_boundary, right_boundary, frame_h)


        curr_time = time.time()
        fps       = 1 / (curr_time - prev_time + 1e-9)
        prev_time = curr_time
        draw_hud(annoted, fps, nav_status, math.degrees(current_yaw_rate), frame_h)

        cv2.imshow("YOLOv5 Boat Navigation", annoted)
        print(f"[{nav_status}]  red={red_ball is not None}  green={green_ball is not None}  black={black_ball is not None}")

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        
        
    cap.release()
    cv2.destroyAllWindows()
    vehicle.close()

    print("Mission Completed")

