#!/usr/bin/env python3
"""
Automatically tiles Gazebo Sim and RViz2 windows side-by-side on display :1.
- Left Half  : Gazebo Sim 3D Warehouse
- Right Half : RViz2 Open-RMF Trajectory & Traffic Map
"""
import os
import sys
import time
import subprocess
import ctypes
from ctypes import c_int, c_ulong, c_char_p, c_void_p, Structure, byref

os.environ['DISPLAY'] = os.environ.get('DISPLAY', ':1')
x11 = ctypes.CDLL('libX11.so.6')
x11.XOpenDisplay.restype = c_void_p
x11.XDefaultRootWindow.restype = c_ulong
x11.XInternAtom.restype = c_ulong

display = x11.XOpenDisplay(os.environ['DISPLAY'].encode('utf-8'))
if not display:
    print(f"Cannot open display {os.environ.get('DISPLAY', ':1')}")
    sys.exit(0)

root = x11.XDefaultRootWindow(display)

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
    ev.xclient.data[0] = 0  # _NET_WM_STATE_REMOVE
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
    gz_win = None
    rviz_win = None
    try:
        out = subprocess.check_output(['xwininfo', '-root', '-tree'], text=True)
        for line in out.splitlines():
            if '("gz-sim-gui"' in line:
                parts = line.strip().split()
                if parts[0].startswith('0x'):
                    gz_win = int(parts[0], 16)
            elif '("rviz2" "rviz2")' in line and 'RViz' in line:
                parts = line.strip().split()
                if parts[0].startswith('0x'):
                    rviz_win = int(parts[0], 16)
    except Exception:
        return False

    if rviz_win:
        unmaximize(rviz_win)
        moveresize(rviz_win, 960, 30, 950, 1010)
        raise_win(rviz_win)

    if gz_win:
        unmaximize(gz_win)
        moveresize(gz_win, 0, 30, 955, 1010)
        raise_win(gz_win)

    return (gz_win is not None and rviz_win is not None)

def main():
    # Poll for up to 30 seconds until both windows appear, then tile them
    start = time.time()
    while time.time() - start < 30:
        if tile_all():
            print("[✓] Gazebo and RViz windows successfully tiled side-by-side.")
            # Ensure Gazebo physics is unpaused
            time.sleep(1.0)
            subprocess.run([
                "gz", "service", "-s", "/world/sim_world/control",
                "--reqtype", "gz.msgs.WorldControl",
                "--reptype", "gz.msgs.Boolean",
                "--timeout", "2000",
                "--req", "pause: false"
            ], check=False)
            break
        time.sleep(1.0)
    x11.XCloseDisplay(display)

if __name__ == '__main__':
    main()
