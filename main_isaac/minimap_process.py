#!/usr/bin/env python3
"""
minimap_process.py
==================
시스템 Python3 (GTK/OpenCV GUI 지원)으로 실행되는 미니맵 뷰어.

stdin → BGR 프레임 수신 → OpenCV 창 표시
stdout → 마우스 우클릭 이벤트 → 스폰 요청 전송

stdin 읽기는 별도 스레드로 처리하여 메인 스레드가
cv2.waitKey() 를 계속 호출해 창을 살아있게 한다.

Protocol (stdin):
  [1 byte type] [4 bytes LE size] [data]
  type 0x01 : BGR frame — data: H(2LE) W(2LE) + BGR bytes
  type 0xFF : shutdown

Protocol (stdout, text):
  SPAWN {wx:.3f} {wy:.3f}\n  — 우클릭 위치의 world 좌표
"""
import sys
import struct
import threading
import queue

import cv2
import numpy as np

WIN = "Warehouse Minimap"

# ── 미니맵과 동일한 좌표 변환 상수 ─────────────────────────────────────
_WX0, _WX1 = -18.0, 24.0
_WY0, _WY1 = -18.0, 18.0
_IW,  _IH  = 900, 660


def _p2w(px: int, py: int):
    wx = px / _IW * (_WX1 - _WX0) + _WX0
    wy = (1.0 - py / _IH) * (_WY1 - _WY0) + _WY0
    return wx, wy


def _on_mouse(event, px, py, flags, param):
    if event == cv2.EVENT_RBUTTONDOWN:
        wx, wy = _p2w(px, py)
        sys.stdout.write(f"SPAWN {wx:.3f} {wy:.3f}\n")
        sys.stdout.flush()
    elif event == cv2.EVENT_MOUSEMOVE:
        wx, wy = _p2w(px, py)
        cv2.setWindowTitle(WIN, f"Warehouse Minimap  [{wx:.1f}, {wy:.1f}]  Right click=spawn box")


# ── 프레임 큐 (reader thread → main thread) ─────────────────────────
_frame_q: queue.Queue = queue.Queue(maxsize=2)
_shutdown = threading.Event()


def _stdin_reader():
    """별도 스레드: stdin 에서 프레임을 읽어 _frame_q 에 쌓는다."""
    stdin = sys.stdin.buffer

    def read_exact(n):
        buf = bytearray()
        while len(buf) < n:
            chunk = stdin.read(n - len(buf))
            if not chunk:
                return None
            buf.extend(chunk)
        return bytes(buf)

    while not _shutdown.is_set():
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
            try:
                frame = np.frombuffer(data[4:], dtype=np.uint8).reshape(h, w, 3).copy()
                # drop oldest frame if queue is full to keep latency low
                try:
                    _frame_q.put_nowait(frame)
                except queue.Full:
                    try:
                        _frame_q.get_nowait()
                    except queue.Empty:
                        pass
                    _frame_q.put_nowait(frame)
            except Exception:
                pass

    _shutdown.set()


# ── 창 생성 + 초기 대기 화면 ─────────────────────────────────────────
cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
cv2.resizeWindow(WIN, 900, 680)
cv2.setWindowTitle(WIN, "Warehouse Minimap  (Right click = spawn box)")
cv2.setMouseCallback(WIN, _on_mouse)

_placeholder = np.full((_IH, _IW, 3), (28, 28, 28), dtype=np.uint8)
cv2.putText(_placeholder, "Waiting for simulation...", (240, _IH // 2),
            cv2.FONT_HERSHEY_SIMPLEX, 0.9, (160, 160, 160), 1, cv2.LINE_AA)
cv2.imshow(WIN, _placeholder)
cv2.waitKey(1)

# ── 리더 스레드 시작 ─────────────────────────────────────────────────
_reader = threading.Thread(target=_stdin_reader, daemon=True)
_reader.start()

# ── 메인 루프: cv2 이벤트 펌프 + 프레임 표시 ────────────────────────
# WND_PROP_AUTOSIZE 를 체크해 창이 닫혔는지 감지한다.
# WND_PROP_VISIBLE 은 일부 GTK 빌드에서 창이 열려있어도 0 을 반환하므로 사용 안 함.
while not _shutdown.is_set():
    try:
        frame = _frame_q.get(timeout=0.03)
        cv2.imshow(WIN, frame)
    except queue.Empty:
        pass

    key = cv2.waitKey(1)
    if key == 27:  # ESC
        break
    # 창이 닫혔으면 -1 반환
    if cv2.getWindowProperty(WIN, cv2.WND_PROP_AUTOSIZE) < 0:
        break

cv2.destroyAllWindows()
