#!/usr/bin/env python3
"""
Automatically tiles Gazebo Sim and RViz2 windows for both rmf_ws and rmf_ws_t
on display :1 into a clean grid:
- Top-Left     : Gazebo Sim (rmf_ws - Edge-AI)
- Bottom-Left  : Gazebo Sim (rmf_ws_t - Traditional)
- Top-Right    : RViz2 (rmf_ws - Edge-AI)
- Bottom-Right : RViz2 (rmf_ws_t - Traditional)
Or if only 2 windows exist, tiles them side-by-side (Left / Right).
"""
import os
import sys
import time
import subprocess
import ctypes
from ctypes import c_int, c_ulong, c_char_p, c_void_p, Structure, byref

os.environ['DISPLAY'] = os.environ.get('DISPLAY', ':1')
try:
    x11 = ctypes.CDLL('libX11.so.6')
    x11.XOpenDisplay.restype = c_void_p
    x11.XDefaultRootWindow.restype = c_ulong
    x11.XInternAtom.restype = c_ulong

    display = x11.XOpenDisplay(os.environ['DISPLAY'].encode('utf-8'))
    if not display:
        sys.exit(0)
    root = x11.XDefaultRootWindow(display)
except Exception:
    sys.exit(0)

class XClientMessageEvent(Structure):
    _fields_ = [
        ('type', c_int), ('serial', c_ulong), ('send_event', c_int),
        ('display', c_void_p), ('window', c_ulong), ('message_type', c_ulong),
        ('format', c_int), ('data', c_ulong * 5)
    ]

class XEvent(ctypes.Union):
    _fields_ = [
        ('type', c_int), ('xclient', XClientMessageEvent), ('pad', c_ulong * 24)
    ]

net_wm_state = x11.XInternAtom(display, b'_NET_WM_STATE', False)
max_vert = x11.XInternAtom(display, b'_NET_WM_STATE_MAXIMIZED_VERT', False)
max_horz = x11.XInternAtom(display, b'_NET_WM_STATE_MAXIMIZED_HORZ', False)
net_moveresize = x11.XInternAtom(display, b'_NET_MOVERESIZE_WINDOW', False)
net_active = x11.XInternAtom(display, b'_NET_ACTIVE_WINDOW', False)

def unmaximize(win):
    ev = XEvent()
    ev.xclient.type = 33
    ev.xclient.send_event = 1
    ev.xclient.display = display
    ev.xclient.window = win
    ev.xclient.message_type = net_wm_state
    ev.xclient.format = 32
    ev.xclient.data[0] = 0
    ev.xclient.data[1] = max_vert
    ev.xclient.data[2] = max_horz
    ev.xclient.data[3] = 1
    ev.xclient.data[4] = 0
    mask = (1 << 19) | (1 << 20)
    x11.XSendEvent(display, root, False, mask, byref(ev))
    x11.XFlush(display)

def moveresize(win, x, y, w, h):
    ev = XEvent()
    ev.xclient.type = 33
    ev.xclient.send_event = 1
    ev.xclient.display = display
    ev.xclient.window = win
    ev.xclient.message_type = net_moveresize
    ev.xclient.format = 32
    flags = (1 << 8) | (1 << 9) | (1 << 10) | (1 << 11) | (1 << 12)
    ev.xclient.data[0] = flags
    ev.xclient.data[1] = x
    ev.xclient.data[2] = y
    ev.xclient.data[3] = w
    ev.xclient.data[4] = h
    mask = (1 << 19) | (1 << 20)
    x11.XSendEvent(display, root, False, mask, byref(ev))
    x11.XFlush(display)

def raise_win(win):
    ev = XEvent()
    ev.xclient.type = 33
    ev.xclient.send_event = 1
    ev.xclient.display = display
    ev.xclient.window = win
    ev.xclient.message_type = net_active
    ev.xclient.format = 32
    ev.xclient.data[0] = 1
    ev.xclient.data[1] = 0
    mask = (1 << 19) | (1 << 20)
    x11.XSendEvent(display, root, False, mask, byref(ev))
    x11.XFlush(display)

def tile_all():
    gz_wins = []
    rviz_wins = []
    try:
        out = subprocess.check_output(['xwininfo', '-root', '-tree'], text=True)
        for line in out.splitlines():
            if '("gz-sim-gui"' in line:
                parts = line.strip().split()
                if parts[0].startswith('0x'):
                    win_id = int(parts[0], 16)
                    if win_id not in gz_wins:
                        gz_wins.append(win_id)
            elif '("rviz2" "rviz2")' in line and 'RViz' in line:
                parts = line.strip().split()
                if parts[0].startswith('0x'):
                    win_id = int(parts[0], 16)
                    if win_id not in rviz_wins:
                        rviz_wins.append(win_id)
    except Exception:
        return False

    if not gz_wins and not rviz_wins:
        return False

    # If 4 windows (2 Gazebo + 2 RViz) -> 2x2 grid
    if len(gz_wins) >= 2 and len(rviz_wins) >= 2:
        # Top-Left: Gazebo 1
        unmaximize(gz_wins[0])
        moveresize(gz_wins[0], 0, 30, 955, 490)
        raise_win(gz_wins[0])

        # Bottom-Left: Gazebo 2
        unmaximize(gz_wins[1])
        moveresize(gz_wins[1], 0, 535, 955, 490)
        raise_win(gz_wins[1])

        # Top-Right: RViz 1
        unmaximize(rviz_wins[0])
        moveresize(rviz_wins[0], 960, 30, 950, 490)
        raise_win(rviz_wins[0])

        # Bottom-Right: RViz 2
        unmaximize(rviz_wins[1])
        moveresize(rviz_wins[1], 960, 535, 950, 490)
        raise_win(rviz_wins[1])
        return True

    # If 2 windows -> Left / Right side-by-side
    if len(gz_wins) >= 1 and len(rviz_wins) >= 1:
        unmaximize(gz_wins[0])
        moveresize(gz_wins[0], 0, 30, 955, 1010)
        raise_win(gz_wins[0])

        unmaximize(rviz_wins[0])
        moveresize(rviz_wins[0], 960, 30, 950, 1010)
        raise_win(rviz_wins[0])
        return True

    return False

def unpause_gz_simulation():
    # Unpause Gazebo for both partitions if active
    for part in ['edge_ai', 'traditional', '']:
        env = os.environ.copy()
        if part:
            env['GZ_PARTITION'] = part
        subprocess.run([
            "gz", "service", "-s", "/world/sim_world/control",
            "--reqtype", "gz.msgs.WorldControl",
            "--reptype", "gz.msgs.Boolean",
            "--timeout", "1500",
            "--req", "pause: false"
        ], env=env, check=False, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

def main():
    start = time.time()
    tiled = False
    while time.time() - start < 45:
        if tile_all():
            tiled = True
            time.sleep(1.0)
            unpause_gz_simulation()
            # If we already have 4 windows, we're fully tiled and done
            out = subprocess.check_output(['xwininfo', '-root', '-tree'], text=True)
            gz_c = out.count('("gz-sim-gui"')
            rviz_c = out.count('("rviz2" "rviz2")')
            if gz_c >= 2 and rviz_c >= 2:
                break
        time.sleep(2.0)
    try:
        x11.XCloseDisplay(display)
    except Exception:
        pass

if __name__ == '__main__':
    main()
