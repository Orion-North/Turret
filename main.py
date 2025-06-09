#!/usr/bin/env python3
"""
Turret Control Panel: Human Detection, Scanning, and Manual Control

Launches a comprehensive GUI control panel by default to configure Arduino port,
camera index, detection confidence, and manual or automatic mode.
"""
import threading
import time
import serial
# OpenCV for image processing and face detection
import cv2
# Haar cascade classifier for face detection
face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + \
    'haarcascade_frontalface_default.xml')
from ultralytics import YOLO
import tkinter as tk
from tkinter import ttk
from serial.tools import list_ports
from PIL import Image, ImageTk
from collections import deque
import json
import os

MODEL_PATH = "yolov8n.pt"
PAN_RANGE_DEG = 120
TILT_RANGE_DEG = 60
AUTO_PAN_LIMIT_DEG = PAN_RANGE_DEG
AUTO_PAN_SPEED_MULT = 12.0
# Macro step degrees for pan/tilt when scanning without detections (amplitude per scan cycle)
PAN_STEP_DEG = 4
TILT_STEP_DEG = 2
# Delay (seconds) with no human detection before initiating no-human scanning
NO_HUMAN_SCAN_DELAY = 1.0
# Angle step for auto-tracking micro-movements (degrees)
MICROSTEP_DEG = 0.5
# Maximum step change for a single update (degrees) to limit overshoot
MAX_PAN_STEP_DEG = 10
MAX_TILT_STEP_DEG = 5
# Proportional gain for auto tracking
PAN_KP = 0.5
TILT_KP = 0.5
# Use finer microstepping for Nema 17 steppers for smoother motion
MICROSTEPPING_FACTOR = 16
STEPS_PER_REV = 200
GEAR_RATIO = 6
BAUD_RATE = 115200
SERIAL_TIMEOUT = 1
TOLERANCE_PIXELS = 10
# Number of frames to smooth detection error for auto tracking
SMOOTHING_WINDOW = 10
PRECISION_DIST_PIXELS = 50

# Microstep increments (degrees) for pan and tilt in auto mode
PAN_MICROSTEP_DEG = MICROSTEP_DEG
TILT_MICROSTEP_DEG = MICROSTEP_DEG

PAN_MICROSTEPPING_FACTOR = MICROSTEPPING_FACTOR
PAN_GEAR_RATIO = 1
TILT_MICROSTEPPING_FACTOR = MICROSTEPPING_FACTOR
TILT_GEAR_RATIO = GEAR_RATIO
EYE_LEVEL_RATIO = 0.25
SETTINGS_FILE = "auto_settings.json"

def pan_angle_to_steps(angle_deg):
    return int(round((angle_deg / 360.0) * STEPS_PER_REV * PAN_MICROSTEPPING_FACTOR * PAN_GEAR_RATIO))

def tilt_angle_to_steps(angle_deg):
    return int(round((angle_deg / 360.0) * STEPS_PER_REV * TILT_MICROSTEPPING_FACTOR * TILT_GEAR_RATIO))

def send_command(ser, command):
    ser.write((command + "\n").encode("utf-8"))
    time.sleep(0.02)

def calibrate_directions(ser):
    """
    Calibrate pan and tilt directions. Moves each axis a few steps and asks the user
    to indicate the actual direction of movement.
    """
    print("Calibrating directions...")
    # Pan calibration
    test_deg = PAN_STEP_DEG
    steps = pan_angle_to_steps(test_deg)
    print(f"Pan calibration: sending PAN{steps}")
    send_command(ser, f"PAN{steps}")
    resp = input("Did the turret move to the right (r) or left (l)? ").lower()
    while resp not in ('r', 'l'):
        resp = input("Please enter 'r' for right or 'l' for left: ").lower()
    pan_sign = 1 if resp == 'r' else -1
    # Return to original position
    send_command(ser, f"PAN{-steps}")

    # Tilt calibration
    test_deg = TILT_STEP_DEG
    steps = tilt_angle_to_steps(test_deg)
    print(f"Tilt calibration: sending TILT{steps}")
    send_command(ser, f"TILT{steps}")
    resp = input("Did the turret move up (u) or down (d)? ").lower()
    while resp not in ('u', 'd'):
        resp = input("Please enter 'u' for up or 'd' for down: ").lower()
    tilt_sign = 1 if resp == 'u' else -1
    # Return to original position
    send_command(ser, f"TILT{-steps}")

    print("Calibration complete.")
    return pan_sign, tilt_sign


def control_panel():
    """
    Comprehensive control panel for turret:
      - Select Arduino port and camera index
      - Set detection confidence threshold
      - Choose mode: scan, manual, or auto
    """
    root = tk.Tk()
    root.title("Turret Control Panel")
    root.configure(bg='black')
    style = ttk.Style(root)
    style.theme_use('default')
    style.configure('.', background='black', foreground='blue')
    style.configure('TLabel', background='black', foreground='blue')
    style.configure('TButton', background='black', foreground='blue')
    style.configure('TFrame', background='black')
    style.configure('TCombobox', fieldbackground='black', background='black', foreground='blue')
    style.configure('TRadiobutton', background='black', foreground='blue')
    style.configure('Horizontal.TScale', background='black')
    style.configure('Vertical.TScale', background='black')

    ports = [p.device for p in list_ports.comports()]
    cam_indices = []
    for i in range(6):
        cap_temp = cv2.VideoCapture(i)
        if cap_temp.isOpened():
            cam_indices.append(i)
            cap_temp.release()

    port_var = tk.StringVar(value=ports[0] if ports else "")
    cam_var = tk.StringVar(value=str(cam_indices[0]) if cam_indices else "0")
    conf_var = tk.DoubleVar(value=0.5)
    mode_var = tk.StringVar(value="scan")  # scan, manual, or auto

    # Controls frame
    ctrl_frame = ttk.Frame(root)
    ctrl_frame.grid(row=0, column=0, sticky='nw', padx=5, pady=5)

    ttk.Label(ctrl_frame, text="Arduino Port:").grid(row=0, column=0, sticky='w')
    port_combo = ttk.Combobox(ctrl_frame, textvariable=port_var, values=ports, state='readonly')
    port_combo.grid(row=0, column=1, padx=5, pady=2)

    ttk.Label(ctrl_frame, text="Camera Index:").grid(row=1, column=0, sticky='w')
    cam_combo = ttk.Combobox(ctrl_frame, textvariable=cam_var, values=[str(i) for i in cam_indices], state='readonly')
    cam_combo.grid(row=1, column=1, padx=5, pady=2)

    ttk.Label(ctrl_frame, text="Confidence Threshold:").grid(row=2, column=0, sticky='w')
    conf_scale = ttk.Scale(ctrl_frame, orient='horizontal', variable=conf_var, from_=0.0, to=1.0)
    conf_scale.grid(row=2, column=1, padx=5, pady=2, sticky='ew')

    ttk.Label(ctrl_frame, text="Mode:").grid(row=3, column=0, sticky='w')
    mode_frame = ttk.Frame(ctrl_frame)
    mode_frame.grid(row=3, column=1, pady=5)
    ttk.Radiobutton(mode_frame, text="Scan", variable=mode_var, value="scan").pack(side='left')
    ttk.Radiobutton(mode_frame, text="Manual", variable=mode_var, value="manual").pack(side='left')
    ttk.Radiobutton(mode_frame, text="Auto", variable=mode_var, value="auto").pack(side='left')

    # Video preview canvas
    canvas = tk.Canvas(root, width=640, height=480, bg='black')
    canvas.grid(row=0, column=1, rowspan=5, padx=5, pady=5)

    stop_event = threading.Event()
    cap = cv2.VideoCapture(int(cam_var.get()))
    timer_id = None

    def on_cam_change(event=None):
        nonlocal cap
        idx = int(cam_var.get())
        if cap.isOpened():
            cap.release()
        cap = cv2.VideoCapture(idx)

    cam_combo.bind('<<ComboboxSelected>>', on_cam_change)

    def update_frame():
        if stop_event.is_set():
            return
        if cap.isOpened():
            ret, frame = cap.read()
            if ret:
                img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                pic = Image.fromarray(img)
                imgtk = ImageTk.PhotoImage(image=pic)
                canvas.imgtk = imgtk
                canvas.create_image(0, 0, anchor='nw', image=imgtk)
        nonlocal timer_id
        timer_id = root.after(30, update_frame)

    def on_start():
        stop_event.set()
        nonlocal timer_id
        if timer_id is not None:
            root.after_cancel(timer_id)
        root.quit()

    def on_cancel():
        port_var.set("")
        stop_event.set()
        nonlocal timer_id
        if timer_id is not None:
            root.after_cancel(timer_id)
        root.quit()

    btn_frame = ttk.Frame(ctrl_frame)
    btn_frame.grid(row=4, column=0, columnspan=2, pady=5)
    ttk.Button(btn_frame, text="Start", command=on_start).pack(side='left', padx=5)
    ttk.Button(btn_frame, text="Cancel", command=on_cancel).pack(side='right', padx=5)

    timer_id = root.after(0, update_frame)
    root.mainloop()
    if cap.isOpened():
        cap.release()
    root.destroy()
    return port_var.get(), int(cam_var.get()), conf_var.get(), mode_var.get()

def manual_mode(ser, cap, model, conf, pan_sign, tilt_sign):
    """
    Manual control mode with YOLO overlay and calibrated arrow controls.
    Arrow keys: pan left/right, tilt up/down based on calibration.
    Press 'q' to quit manual mode.
    """
    pan_angle = 0
    tilt_angle = 0
    window_name = "Manual Control"
    cv2.namedWindow(window_name)
    print("Manual mode: YOLO overlay active. Use arrow keys. Press 'q' to quit.")

    left_keys = (81, 65361, 2424832)
    right_keys = (83, 65363, 2555904)
    up_keys = (82, 65362, 2490368)
    down_keys = (84, 65364, 2621440)

    while True:
        ret, frame = cap.read()
        if not ret:
            continue
        # detect faces
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        faces = face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5)
        h, w = frame.shape[:2]
        center = (w // 2, h // 2)
        cv2.drawMarker(frame, center, (255, 0, 0),
                       markerType=cv2.MARKER_CROSS, markerSize=20, thickness=2)
        if len(faces) > 0:
            x, y, fw, fh = faces[0]
            cx = x + fw // 2
            cy = y + fh // 2
            cv2.rectangle(frame, (x, y), (x + fw, y + fh), (0, 255, 0), 2)
            cv2.line(frame, center, (cx, cy), (0, 255, 0), 2)
            cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
            dist = int(((cx - center[0]) ** 2 + (cy - center[1]) ** 2) ** 0.5)
            cv2.putText(frame, f"Dist: {dist}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.imshow(window_name, frame)
        key = cv2.waitKeyEx(30)
        if key == ord('q'):
            break
        elif key in left_keys or key in right_keys:
            angle_sign = -1 if key in left_keys else 1
            next_pan = pan_angle + angle_sign * PAN_STEP_DEG
            if abs(next_pan) <= PAN_RANGE_DEG:
                steps = pan_angle_to_steps(PAN_STEP_DEG)
                send_command(ser, f"PAN{angle_sign * steps * pan_sign}")
                pan_angle = next_pan
            else:
                print("Pan limit reached")
        elif key in up_keys or key in down_keys:
            angle_sign = 1 if key in up_keys else -1
            next_tilt = tilt_angle + angle_sign * TILT_STEP_DEG
            if abs(next_tilt) <= TILT_RANGE_DEG:
                steps = tilt_angle_to_steps(TILT_STEP_DEG)
                send_command(ser, f"TILT{angle_sign * steps * tilt_sign}")
                tilt_angle = next_tilt
            else:
                print("Tilt limit reached")
    cv2.destroyWindow(window_name)

def auto_mode(ser, cap, model, conf, pan_sign, tilt_sign):
    """
    Auto tracking mode with live settings: moves turret incrementally towards detected target.
    Press 'q' or close the settings panel to quit auto mode.
    """
    window_name = "Auto Tracking"
    cv2.namedWindow(window_name)
    print("Auto mode: auto-tracking enabled. Press 'q' or close the settings panel to quit.")

    if os.path.exists(SETTINGS_FILE):
        try:
            with open(SETTINGS_FILE, 'r') as f:
                saved = json.load(f)
            conf = saved.get('conf', conf)
            tol_default = saved.get('tolerance', TOLERANCE_PIXELS)
            smooth_default = saved.get('smoothing_window', SMOOTHING_WINDOW)
            pan_default = saved.get('pan_microstep', PAN_MICROSTEP_DEG)
            tilt_default = saved.get('tilt_microstep', TILT_MICROSTEP_DEG)
            target_default = saved.get('target_mode', 'first')
        except Exception as e:
            print(f"Error loading settings: {e}")
            tol_default = TOLERANCE_PIXELS
            smooth_default = SMOOTHING_WINDOW
            pan_default = PAN_MICROSTEP_DEG
            tilt_default = TILT_MICROSTEP_DEG
            target_default = 'first'
    else:
        tol_default = TOLERANCE_PIXELS
        smooth_default = SMOOTHING_WINDOW
        pan_default = PAN_MICROSTEP_DEG
        tilt_default = TILT_MICROSTEP_DEG
        target_default = 'first'

    # Settings panel
    settings_root = tk.Tk()
    settings_root.title("Settings Panel")
    settings_root.configure(bg='black')
    style = ttk.Style(settings_root)
    style.theme_use('default')
    style.configure('.', background='black', foreground='blue')
    style.configure('TLabel', background='black', foreground='blue')
    style.configure('TButton', background='black', foreground='blue')
    style.configure('TFrame', background='black')
    style.configure('TCombobox', fieldbackground='black', background='black', foreground='blue')
    style.configure('TRadiobutton', background='black', foreground='blue')
    style.configure('Horizontal.TScale', background='black')
    style.configure('Vertical.TScale', background='black')

    def save_settings():
        settings = {
            "conf": conf_var.get(),
            "tolerance": tol_var.get(),
            "smoothing_window": smooth_var.get(),
            "pan_microstep": pan_step_var.get(),
            "tilt_microstep": tilt_step_var.get(),
            "target_mode": target_var.get()
        }
        try:
            with open(SETTINGS_FILE, 'w') as f:
                json.dump(settings, f, indent=4)
            print(f"Settings saved to {SETTINGS_FILE}")
        except Exception as e:
            print(f"Error saving settings: {e}")

    conf_var = tk.DoubleVar(value=conf)
    tol_var = tk.IntVar(value=tol_default)
    smooth_var = tk.IntVar(value=smooth_default)
    pan_step_var = tk.DoubleVar(value=pan_default)
    tilt_step_var = tk.DoubleVar(value=tilt_default)
    target_var = tk.StringVar(value=target_default)

    frm = ttk.Frame(settings_root)
    frm.pack(padx=10, pady=10, fill='x')

    ttk.Label(frm, text="Confidence Threshold").pack(fill='x')
    ttk.Scale(frm, orient='horizontal', variable=conf_var, from_=0.0, to=1.0).pack(fill='x')

    ttk.Label(frm, text="Tolerance (pixels)").pack(fill='x')
    ttk.Scale(frm, orient='horizontal', variable=tol_var, from_=0, to=100).pack(fill='x')

    ttk.Label(frm, text="Smoothing Window").pack(fill='x')
    ttk.Scale(frm, orient='horizontal', variable=smooth_var, from_=1, to=30).pack(fill='x')

    ttk.Label(frm, text="Pan Microstep (deg)").pack(fill='x')
    ttk.Scale(frm, orient='horizontal', variable=pan_step_var, from_=0.1, to=5.0).pack(fill='x')

    ttk.Label(frm, text="Tilt Microstep (deg)").pack(fill='x')
    ttk.Scale(frm, orient='horizontal', variable=tilt_step_var, from_=0.1, to=5.0).pack(fill='x')

    ttk.Label(frm, text="Target Mode").pack(fill='x')
    ttk.Combobox(frm, textvariable=target_var,
                 values=["first", "leftmost", "rightmost", "closest"], state='readonly').pack(fill='x')

    btn_frame = ttk.Frame(frm)
    btn_frame.pack(pady=5)
    ttk.Button(btn_frame, text="Save Settings", command=save_settings).pack(side='left', padx=5)
    ttk.Button(btn_frame, text="Stop Auto", command=settings_root.destroy).pack(side='left', padx=5)

    pan_angle = 0
    tilt_angle = 0
    tilt_scan_dir = -1
    last_window = smooth_var.get()
    err_x_history = deque(maxlen=last_window)
    err_y_history = deque(maxlen=last_window)
    last_human_time = time.time()

    while True:
        # Process settings GUI events
        try:
            settings_root.update_idletasks()
            settings_root.update()
        except tk.TclError:
            # Settings window closed
            break

        ret, frame = cap.read()
        if not ret:
            continue

        conf_val = conf_var.get()
        tolerance = tol_var.get()
        window_size = smooth_var.get()
        pan_micro = pan_step_var.get()
        tilt_micro = tilt_step_var.get()

        if window_size != last_window:
            err_x_history = deque(err_x_history, maxlen=window_size)
            err_y_history = deque(err_y_history, maxlen=window_size)
            last_window = window_size

        curr_time = time.time()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        faces = face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5)
        h, w = frame.shape[:2]
        center = (w // 2, h // 2)
        cv2.drawMarker(frame, center, (255, 0, 0),
                       markerType=cv2.MARKER_CROSS, markerSize=20, thickness=2)
        if len(faces) > 0:
            last_human_time = curr_time
            mode = target_var.get()
            if mode == 'leftmost':
                x, y, fw, fh = min(faces, key=lambda f: f[0])
            elif mode == 'rightmost':
                x, y, fw, fh = max(faces, key=lambda f: f[0] + f[2])
            elif mode == 'closest':
                x, y, fw, fh = min(faces, key=lambda f: abs((f[0] + f[2] / 2) - center[0]))
            else:
                x, y, fw, fh = faces[0]
            cx = x + fw // 2
            cy = y + fh // 2
            cv2.rectangle(frame, (x, y), (x + fw, y + fh), (0, 255, 0), 2)
            cv2.line(frame, center, (cx, cy), (0, 255, 0), 2)
            cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
            err_x = cx - center[0]
            err_y = center[1] - cy
            err_x_history.append(err_x)
            err_y_history.append(err_y)
            smoothed_err_x = sum(err_x_history) / len(err_x_history)
            smoothed_err_y = sum(err_y_history) / len(err_y_history)

            if abs(smoothed_err_x) > tolerance:
                # Proportional control scaled to screen size
                pan_step = (PAN_RANGE_DEG * (smoothed_err_x / (w / 2))) * PAN_KP
                pan_step = max(pan_micro, min(abs(pan_step), MAX_PAN_STEP_DEG)) * (1 if pan_step >= 0 else -1)
                if abs(pan_angle + pan_step) <= AUTO_PAN_LIMIT_DEG:
                    steps = pan_angle_to_steps(abs(pan_step))
                    pan_dir = 1 if pan_step > 0 else -1
                    send_command(ser, f"PAN{pan_dir * steps * pan_sign}")
                    pan_angle += pan_step

            if abs(smoothed_err_y) > tolerance:
                tilt_step = (TILT_RANGE_DEG * (smoothed_err_y / (h / 2))) * TILT_KP
                tilt_step = max(tilt_micro, min(abs(tilt_step), MAX_TILT_STEP_DEG)) * (1 if tilt_step >= 0 else -1)
                if abs(tilt_angle + tilt_step) <= TILT_RANGE_DEG:
                    steps = tilt_angle_to_steps(abs(tilt_step))
                    tilt_dir = 1 if tilt_step > 0 else -1
                    send_command(ser, f"TILT{tilt_dir * steps * tilt_sign}")
                    tilt_angle += tilt_step

        elif curr_time - last_human_time >= NO_HUMAN_SCAN_DELAY:
            scan_step = tilt_micro
            if abs(tilt_angle + tilt_scan_dir * scan_step) > TILT_RANGE_DEG:
                tilt_scan_dir *= -1
            steps = tilt_angle_to_steps(scan_step)
            send_command(ser, f"TILT{tilt_scan_dir * steps * tilt_sign}")
            tilt_angle += tilt_scan_dir * scan_step

        cv2.imshow(window_name, frame)
        key = cv2.waitKeyEx(30)
        if key == ord('q'):
            break

    settings_root.destroy()
    cv2.destroyWindow(window_name)

def main():
    port, camera, conf, mode = control_panel()
    if not port:
        return

    ser = serial.Serial(port, BAUD_RATE, timeout=SERIAL_TIMEOUT)
    time.sleep(2)
    # Calibrate directions for manual and auto tracking modes
    if mode in ('manual', 'auto'):
        pan_sign, tilt_sign = calibrate_directions(ser)
    else:
        pan_sign, tilt_sign = 1, 1
    cap = cv2.VideoCapture(camera)
    if not cap.isOpened():
        print(f"Error: Cannot open camera {camera}")
        return

    model = YOLO(MODEL_PATH)
    if mode == 'manual':
        manual_mode(ser, cap, model, conf, pan_sign, tilt_sign)
        cap.release()
        ser.close()
        return
    if mode == 'auto':
        auto_mode(ser, cap, model, conf, pan_sign, tilt_sign)
        cap.release()
        ser.close()
        return
    pan_angle = 0
    pan_dir = 1
    tilt_angle = 0
    tilt_dir = 1
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                continue
            # detect faces
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            faces = face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5)
            if len(faces) > 0:
                # draw center crosshair and annotate detected face
                h, w = frame.shape[:2]
                center = (w // 2, h // 2)
                cv2.drawMarker(frame, center, (255, 0, 0), markerType=cv2.MARKER_CROSS,
                               markerSize=20, thickness=2)
                x, y, fw, fh = faces[0]
                cx = x + fw // 2
                cy = y + fh // 2
                cv2.rectangle(frame, (x, y), (x + fw, y + fh), (0, 255, 0), 2)
                cv2.line(frame, center, (cx, cy), (0, 255, 0), 2)
                cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
                dist = int(((cx - center[0]) ** 2 + (cy - center[1]) ** 2) ** 0.5)
                cv2.putText(frame, f"Dist: {dist}", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                cv2.imshow("Detection", frame)
                cv2.waitKey(0)
                print("Face detected!")
                send_command(ser, "BEEP")
                break
            next_pan = pan_angle + pan_dir * PAN_STEP_DEG
            if abs(next_pan) > PAN_RANGE_DEG:
                pan_dir *= -1
                next_pan = pan_angle + pan_dir * PAN_STEP_DEG
                next_tilt = tilt_angle + tilt_dir * TILT_STEP_DEG
                if abs(next_tilt) > TILT_RANGE_DEG:
                    tilt_dir *= -1
                    next_tilt = tilt_angle + tilt_dir * TILT_STEP_DEG
                delta_tilt = next_tilt - tilt_angle
                steps_tilt = tilt_angle_to_steps(abs(delta_tilt))
                cmd_tilt = f"TILT{steps_tilt if delta_tilt >= 0 else -steps_tilt}"
                send_command(ser, cmd_tilt)
                tilt_angle = next_tilt
            delta_pan = next_pan - pan_angle
            steps_pan = pan_angle_to_steps(abs(delta_pan))
            cmd_pan = f"PAN{steps_pan if delta_pan >= 0 else -steps_pan}"
            send_command(ser, cmd_pan)
            pan_angle = next_pan
    except KeyboardInterrupt:
        print("Interrupted by user")
    finally:
        cap.release()
        ser.close()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()