"""
main_isaac/work_signals.py
===========================
M0609 → IW Hub 직접 Python 신호 모듈.

ROS2 spin 타이밍 문제를 우회하는 동일 프로세스 내 신호 카운터.
M0609 가 픽앤플레이스 완료 시 signal() 호출 → IW Hub 가 FSM 에서 get() 확인.
"""
import threading

_lock   = threading.Lock()
_counts: dict = {}   # {section_name: count}


def signal(section: str) -> None:
    """M0609 이 section 픽앤플레이스 완료 시 호출."""
    with _lock:
        _counts[section] = _counts.get(section, 0) + 1


def get(section: str) -> int:
    """IW Hub 가 현재 누적 완료 횟수를 확인."""
    with _lock:
        return _counts.get(section, 0)


def reset(section: str) -> None:
    """IW Hub 가 동작 시작 시 초기화."""
    with _lock:
        _counts[section] = 0
