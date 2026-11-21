import socket
import sys
import time

try:
    import vgamepad as vg
except ImportError:
    vg = None

PORT = 4210
FAILSAFE_S = 0.5
DEADBAND = 0.03
PRINT_VALUES = True
PRINT_INTERVAL_S = 0.1          # 10 Hz readout

# flip these if a stick moves the wrong way in the sim
INVERT_YAW, INVERT_PITCH, INVERT_ROLL = False, False, False

if vg:
    BTN_MAP = [  # ladder bit 0..7 -> gamepad button
        vg.XUSB_BUTTON.XUSB_GAMEPAD_X,
        vg.XUSB_BUTTON.XUSB_GAMEPAD_Y,
        vg.XUSB_BUTTON.XUSB_GAMEPAD_LEFT_SHOULDER,
        vg.XUSB_BUTTON.XUSB_GAMEPAD_RIGHT_SHOULDER,
        vg.XUSB_BUTTON.XUSB_GAMEPAD_START,
        vg.XUSB_BUTTON.XUSB_GAMEPAD_BACK,
        vg.XUSB_BUTTON.XUSB_GAMEPAD_LEFT_THUMB,
        vg.XUSB_BUTTON.XUSB_GAMEPAD_RIGHT_THUMB,
    ]
    GEAR_BTN = vg.XUSB_BUTTON.XUSB_GAMEPAD_A
    STAB_BTN = vg.XUSB_BUTTON.XUSB_GAMEPAD_B


def axis(v, invert=False):
    """0..255 (centre 128) -> -1.0..1.0"""
    x = (v - 128) / (127.0 if v >= 128 else 128.0)
    x = max(-1.0, min(1.0, x))
    if abs(x) < DEADBAND:
        x = 0.0
    return -x if invert else x


def to_stick(x):
    return int(max(-1.0, min(1.0, x)) * 32767)


def set_button(pad, btn, pressed):
    (pad.press_button if pressed else pad.release_button)(button=btn)


def main():
    pad = vg.VX360Gamepad() if vg else None
    if not pad:
        print("vgamepad not installed - print-only mode")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", PORT))
    sock.settimeout(0.1)

    last_rx = time.time()
    last_print = 0.0
    last_seq = -1
    failsafe = False
    pkt_count = 0
    print(f"Listening on UDP {PORT} ...")

    while True:
        try:
            data, addr = sock.recvfrom(256)
        except socket.timeout:
            if time.time() - last_rx > FAILSAFE_S and not failsafe:
                failsafe = True
                if pad:
                    pad.reset()
                    pad.left_joystick(x_value=0, y_value=-32768)  # throttle 0
                    pad.update()
                print("\nLink lost - failsafe")
            continue

        try:
            f = data.decode().strip().split(",")
            seq, thr, yaw, pitch, roll, gear, stab, live, btn = map(int, f)
        except (ValueError, UnicodeDecodeError):
            print(f"\nBad packet from {addr[0]}: {data!r}")
            continue

        if seq <= last_seq and last_seq - seq < 1000:   # drop stale/out-of-order
            continue
        last_seq = seq
        last_rx = time.time()
        pkt_count += 1
        if failsafe:
            failsafe = False
            print("\nLink restored")
        
        throttle = thr if live else 0
        ly = to_stick(throttle / 50.0 - 1.0)
        lx = to_stick(axis(yaw, INVERT_YAW))
        rx = to_stick(axis(roll, INVERT_ROLL))
        ry = to_stick(axis(pitch, INVERT_PITCH))

        if pad:
            pad.left_joystick(x_value=lx, y_value=ly)
            pad.right_joystick(x_value=rx, y_value=ry)
            set_button(pad, GEAR_BTN, gear)
            set_button(pad, STAB_BTN, stab)
            for i, b in enumerate(BTN_MAP):
                set_button(pad, b, btn & (1 << i))
            pad.update()

        now = time.time()
        if PRINT_VALUES and now - last_print >= PRINT_INTERVAL_S:
            last_print = now
            line = (
                f"#{seq:<7} thr={throttle:3d}% yaw={yaw:3d} pit={pitch:3d} rol={roll:3d} | "
                f"gear={'UP ' if gear else 'DWN'} stab={'ON ' if stab else 'OFF'} "
                f"armed={'Y' if live else 'N'} btn={btn:08b} | "
                f"LX={lx:6d} LY={ly:6d} RX={rx:6d} RY={ry:6d}"
            )
            sys.stdout.write("\r" + line)
            sys.stdout.flush()


if __name__ == "__main__":
    main()