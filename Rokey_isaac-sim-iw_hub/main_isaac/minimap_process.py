#!/usr/bin/env python3
"""
minimap_process.py
==================
시스템 Python3 (GTK/OpenCV GUI 지원)으로 실행되는 미니맵 뷰어.

stdin → BGR 프레임 수신 → OpenCV 창 표시
stdout → 마우스 우클릭 이벤트 → 스폰 요청 전송

Protocol (stdin):
  [1 byte type] [4 bytes LE size] [data]
  type 0x01 : BGR frame — data: H(2LE) W(2LE) + BGR bytes
  type 0xFF : shutdown

Protocol (stdout, text):
  SPAWN {wx:.3f} {wy:.3f}\\n  — 우클릭 위치의 world 좌표
"""
import sys
import struct
import cv2
import numpy as np

WIN = "Warehouse Minimap"

# ── 미니맵과 동일한 좌표 변환 상수 ─────────────────────────────────────
_WX0, _WX1 = -18.0, 24.0
_WY0, _WY1 = -18.0, 18.0
_IW,  _IH  = 900, 660


def _p2w(px: int, py: int):
    """픽셀 좌표 → world (x, y)."""
    wx = px / _IW * (_WX1 - _WX0) + _WX0
    wy = (1.0 - py / _IH) * (_WY1 - _WY0) + _WY0
    return wx, wy


def _on_mouse(event, px, py, flags, param):
    """우클릭: world 좌표를 stdout 에 출력 → 메인 프로세스로 스폰 요청."""
    if event == cv2.EVENT_RBUTTONDOWN:
        wx, wy = _p2w(px, py)
        sys.stdout.write(f"SPAWN {wx:.3f} {wy:.3f}\n")
        sys.stdout.flush()
    elif event == cv2.EVENT_MOUSEMOVE:
        # 툴팁: 현재 hover 좌표를 타이틀 바에 표시
        wx, wy = _p2w(px, py)
        cv2.setWindowTitle(WIN, f"Warehouse Minimap  [{wx:.1f}, {wy:.1f}]  우클릭=Pod 스폰")


stdin = sys.stdin.buffer

cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
cv2.resizeWindow(WIN, 900, 680)
cv2.setWindowTitle(WIN, "Warehouse Minimap  (우클릭 = Pod Stack 스폰)")
cv2.setMouseCallback(WIN, _on_mouse)


def read_exact(n: int):
    buf = bytearray()
    while len(buf) < n:
        chunk = stdin.read(n - len(buf))
        if not chunk:
            return None
        buf.extend(chunk)
    return bytes(buf)


while True:
    header = read_exact(5)
    if header is None:
        break

    msg_type = header[0]
    size = struct.unpack('<I', header[1:5])[0]

    if msg_type == 0xFF:
        break

    data = read_exact(size) if size > 0 else b""
    if data is None:
        break

    if msg_type == 0x01 and len(data) >= 4:
        h, w = struct.unpack('<HH', data[:4])
        frame = np.frombuffer(data[4:], dtype=np.uint8).reshape(h, w, 3).copy()
        cv2.imshow(WIN, frame)

    cv2.waitKey(1)

cv2.destroyAllWindows()
