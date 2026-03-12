import math
import cv2
import torch
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

def get_effective_area(box_np, frame, edge_thresh=10):
    frame_h, frame_w = frame.shape[:2]
    x1, y1, x2, y2  = box_np[0], box_np[1], box_np[2], box_np[3]

    visible_w = x2 - x1
    visible_h = y2 - y1

    cut_left   = x1 < edge_thresh
    cut_right  = x2 > frame_w - edge_thresh
    cut_top    = y1 < edge_thresh
    cut_bottom = y2 > frame_h - edge_thresh

    # Jika terpotong satu sisi → mirror sisi yang terlihat penuh
    estimated_w = visible_w * 2 if (cut_left  != cut_right)  else visible_w
    estimated_h = visible_h * 2 if (cut_top   != cut_bottom) else visible_h

    return estimated_w * estimated_h

def is_centered_in_frame(box_np, frame, margin=0.30):
    frame_h, frame_w = frame.shape[:2]
    x1, y1, x2, y2  = box_np[0], box_np[1], box_np[2], box_np[3]
    cx = (x1 + x2) / 2
    cy = (y1 + y2) / 2
    return (frame_w * margin < cx < frame_w * (1 - margin) and
            frame_h * margin < cy < frame_h * (1 - margin))


def pick_closest(candidates, frame):
    return max(candidates, key=lambda b: get_effective_area(b, frame))

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
    

def ball_zone(ball):
    x1, y1, x2, y2 = ball[0], ball[1], ball[2], ball[3]
    cx = (x1 + x2) / 2
    return get_zone(cx, left_boundary, right_boundary)


model = torch.hub.load(
    'ultralytics/yolov5', 
    'custom', 
    path='/home/risanws/learning/kuliah/bengawan/final-project/ball-detection-yolov5.pt', 
    source='local'
)

model.conf = 0.9  # confidence threshold

cap = cv2.VideoCapture(1)
arm_and_takeoff(0)

prev_time = time.time()  # Waktu sebelumnya untuk menghitung FPS

CENTER_HALF_WIDTH = 100
MAX_YAW_NORMAL    = math.radians(30)
MAX_YAW_CLOSE     = math.radians(40)
YAW_STEP          = math.radians(3)
GROUND_SPEED      = 2
BLACK_CLOSE_RATIO = 0.08   # 8% luas frame

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

