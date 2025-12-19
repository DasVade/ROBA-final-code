import socket
import time
import pygame
import sys
import threading

ROBOT_IP = "192.168.4.1"
ROBOT_PORT = 4210
LOCAL_PORT = 5005
AXIS_DEADZONE = 0.25

BTN_A  = 0
BTN_B  = 1
BTN_LB = 4    # Left shoulder (LB) for servo toggle
BTN_RB = 5    # RB for wall-follow start

mode_lock = threading.Lock()
mode = "JOYSTICK"   # JOYSTICK / WALL / VIVE


def init_joystick():
    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        print("No joystick detected!")
        sys.exit(1)
    js = pygame.joystick.Joystick(0)
    js.init()
    print(f"Using joystick: {js.get_name()}")
    print(f"Axes: {js.get_numaxes()}, Buttons: {js.get_numbuttons()}")
    return js


def get_direction_from_axes(axis_x, axis_y):
    if abs(axis_x) < AXIS_DEADZONE:
        axis_x = 0.0
    if abs(axis_y) < AXIS_DEADZONE:
        axis_y = 0.0

    if axis_x == 0.0 and axis_y == 0.0:
        return 'S'

    if abs(axis_y) >= abs(axis_x):
        return 'F' if axis_y < 0 else 'B'
    return 'L' if axis_x < 0 else 'R'


def send(sock, s: str):
    sock.sendto((s.strip() + "\n").encode(), (ROBOT_IP, ROBOT_PORT))


def recv_thread(sock, stop_event):
    sock.settimeout(0.2)
    while not stop_event.is_set():
        try:
            data, _ = sock.recvfrom(4096)
            txt = data.decode(errors="ignore").strip()
            if txt:
                print(f"[ESP->PC] {txt}")
        except socket.timeout:
            pass
        except OSError:
            break


def input_thread(sock, stop_event):
    """
    Always running; only acts when mode == VIVE.
    Lets you type in the terminal while pygame loop runs.
    
    NEW: You can send 1 or 2 targets:
      - Single target: x,y
      - Two targets:   x1,y1 x2,y2
      - Attack commands: AL, AH, AN
    """
    global mode
    while not stop_event.is_set():
        try:
            line = sys.stdin.readline()
            if not line:
                continue
            line = line.strip()
            if not line:
                continue

            with mode_lock:
                m = mode

            if m != "VIVE":
                print("[INPUT] Ignored (not in VIVE mode). Press B to enter VIVE.")
                continue

            # Check for attack commands
            if line.upper() == "AL":
                send(sock, "AL")
                print("[VIVE] Sent AL (Attack Lower Tower)")
                continue
            
            if line.upper() == "AH":
                send(sock, "AH")
                print("[VIVE] Sent AH (Attack High Tower)")
                continue
            
            if line.upper() == "AN":
                send(sock, "AN")
                print("[VIVE] Sent AN (Attack Nexus)")
                continue

            # Parse targets: either "x,y" or "x1,y1 x2,y2"
            parts = line.split()
            
            if len(parts) == 1:
                # Single target: "x,y"
                if "," in parts[0]:
                    sx, sy = parts[0].split(",", 1)
                    x = float(sx.strip())
                    y = float(sy.strip())
                    send(sock, f"TARGET:{x},{y}")
                    print(f"[VIVE] Sent single TARGET:{x},{y}")
                else:
                    print("[VIVE] Format: x,y  OR  x1,y1 x2,y2  OR  AL/AH/AN")
                    
            elif len(parts) == 2:
                # Two targets: "x1,y1 x2,y2"
                try:
                    sx1, sy1 = parts[0].split(",", 1)
                    sx2, sy2 = parts[1].split(",", 1)
                    x1 = float(sx1.strip())
                    y1 = float(sy1.strip())
                    x2 = float(sx2.strip())
                    y2 = float(sy2.strip())
                    send(sock, f"TARGET2:{x1},{y1},{x2},{y2}")
                    print(f"[VIVE] Sent dual TARGET2:{x1},{y1} -> {x2},{y2}")
                except ValueError:
                    print("[VIVE] Format: x1,y1 x2,y2  (two targets separated by space)")
            else:
                print("[VIVE] Format: x,y  OR  x1,y1 x2,y2  OR  AL/AH/AN")

        except Exception as e:
            print(f"[INPUT] error: {e}")


def main():
    global mode

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", LOCAL_PORT))

    js = init_joystick()
    stop_event = threading.Event()

    threading.Thread(target=recv_thread, args=(sock, stop_event), daemon=True).start()
    threading.Thread(target=input_thread, args=(sock, stop_event), daemon=True).start()

    print("Controls:")
    print("  A: toggle WALL <-> JOYSTICK")
    print("  B: toggle VIVE <-> JOYSTICK")
    print("  RB: start wall-follow motion (WALL mode only)")
    print("  LB: servo toggle (sends 'T') -- JOYSTICK ONLY")
    print("VIVE typing (when in VIVE mode):")
    print("  Single target:  1000,2000")
    print("  Dual targets:   1000,2000 3000,4000")
    print("  Attack:         AL  or  AH  or  AN")

    last_move_cmd = None
    last_a = last_b = last_rb = 0
    last_lb = 0

    try:
        while True:
            pygame.event.pump()

            a  = js.get_button(BTN_A)
            b  = js.get_button(BTN_B)
            lb = js.get_button(BTN_LB)
            rb = js.get_button(BTN_RB)

            # Toggle WALL <-> JOYSTICK
            if a == 1 and last_a == 0:
                with mode_lock:
                    if mode != "WALL":
                        mode = "WALL"
                        send(sock, "W")
                        print("[MODE] -> WALL_FOLLOW")
                    else:
                        mode = "JOYSTICK"
                        send(sock, "J")
                        print("[MODE] -> JOYSTICK")
                last_move_cmd = None
            last_a = a

            # Toggle VIVE <-> JOYSTICK
            if b == 1 and last_b == 0:
                with mode_lock:
                    if mode != "VIVE":
                        mode = "VIVE"
                        send(sock, "V")
                        print("[MODE] -> VIVE (type: x,y  OR  x1,y1 x2,y2  OR  AL)")
                    else:
                        mode = "JOYSTICK"
                        send(sock, "J")
                        print("[MODE] -> JOYSTICK")
                last_move_cmd = None
            last_b = b

            # RB = start wall-follow motion (WALL mode only)
            if rb == 1 and last_rb == 0:
                with mode_lock:
                    m = mode
                if m == "WALL":
                    send(sock, "G")
                    print("[WALL] RB -> START")
            last_rb = rb

            with mode_lock:
                m = mode

            # Only act on LB + joystick motion in JOYSTICK mode
            if m != "JOYSTICK":
                last_lb = lb
                time.sleep(0.02)
                continue

            # LB servo toggle
            if lb == 1 and last_lb == 0:
                send(sock, "T")
                print("[SERVO] LB -> TOGGLE (sent 'T')")
            last_lb = lb

            axis_x = js.get_axis(0)
            axis_y = js.get_axis(1)
            cmd = get_direction_from_axes(axis_x, axis_y)

            if cmd != last_move_cmd:
                send(sock, cmd)
                print(f"[JOY] Sent {cmd} (x={axis_x:.2f}, y={axis_y:.2f})")
                last_move_cmd = cmd

            time.sleep(0.02)

    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        stop_event.set()
        try:
            send(sock, "S")
        except:
            pass
        sock.close()
        pygame.quit()


if __name__ == "__main__":
    main()