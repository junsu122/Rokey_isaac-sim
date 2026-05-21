"""
Firebase Firestore 초기화 및 공통 유틸.

사전 준비:
  1. Firebase 콘솔 → 프로젝트 설정 → 서비스 계정 → 새 비공개 키 생성
  2. 다운로드한 JSON 파일을 config/serviceAccountKey.json 에 배치
  3. pip install firebase-admin
"""

import firebase_admin
from firebase_admin import credentials, firestore
from pathlib import Path
from datetime import datetime, timezone

KEY_PATH = Path(__file__).parent.parent / "robot" / "config" / "serviceAccountKey.json"


def init_firebase() -> firestore.Client:
    """Firebase 앱 초기화 (중복 초기화 방지)."""
    if not firebase_admin._apps:
        if not KEY_PATH.exists():
            raise FileNotFoundError(
                f"서비스 계정 키 없음: {KEY_PATH}\n"
                "Firebase 콘솔 → 프로젝트 설정 → 서비스 계정 → 새 비공개 키 생성 후 배치하세요."
            )
        cred = credentials.Certificate(str(KEY_PATH))
        firebase_admin.initialize_app(cred)

    return firestore.client()


def now_ts() -> datetime:
    """UTC 타임스탬프 반환."""
    return datetime.now(timezone.utc)
