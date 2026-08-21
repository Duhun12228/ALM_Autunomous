#!/usr/bin/env bash
# Piper TTS 설치 — voice_announcer 가 쓰는 음성 합성기.
#
# 왜 prebuilt 바이너리인가:
#   이 젯슨에는 pip 이 없다 (`python3 -m pip` → No module named pip). `pip install
#   piper-tts` 경로는 onnxruntime 까지 aarch64 로 끌어와야 해서 재현성이 나쁘다.
#   릴리스 바이너리는 onnxruntime 과 espeak-ng-data 를 통째로 담고 있어 압축만 풀면 끝난다.
#
# 왜 /opt 가 아닌가:
#   이 머신은 무암호 sudo 가 없다. 홈 아래에 두면 루트 없이 설치되고, 동작은 같다.
#
#   ./scripts/install_piper.sh              # 기본 위치에 설치
#   ALM_VOICE_PREFIX=/opt/piper sudo -E ./scripts/install_piper.sh   # 시스템 전역
set -euo pipefail

PREFIX="${ALM_VOICE_PREFIX:-$HOME/.local/share/alm-voice}"
PIPER_TAG="2023.11.14-2"
VOICE="en_US-lessac-low"
VOICE_URL="https://huggingface.co/rhasspy/piper-voices/resolve/main/en/en_US/lessac/low"

echo "설치 위치: $PREFIX"
mkdir -p "$PREFIX/voices"

# ── 1. piper 바이너리 ────────────────────────────────────────────────────
if [ -x "$PREFIX/piper/piper" ]; then
  echo "✓ piper 이미 있음 — 건너뜀"
else
  echo "→ piper $PIPER_TAG (aarch64, 26 MB) 내려받는 중…"
  curl -fL --retry 3 \
    "https://github.com/rhasspy/piper/releases/download/${PIPER_TAG}/piper_linux_aarch64.tar.gz" \
    | tar xz -C "$PREFIX"
  # 릴리스 tar 는 piper/ 디렉터리를 그대로 담고 있다
  test -x "$PREFIX/piper/piper" || { echo "✗ piper 실행파일이 없다"; exit 1; }
  echo "✓ piper 설치됨"
fi

# ── 2. 음성 모델 ─────────────────────────────────────────────────────────
# .onnx 와 .onnx.json 이 **둘 다** 있어야 한다. json 에 샘플레이트와 음소 설정이
# 들어 있어서, 빠지면 piper 가 그냥 죽는다.
for f in "${VOICE}.onnx" "${VOICE}.onnx.json"; do
  if [ -s "$PREFIX/voices/$f" ]; then
    echo "✓ $f 이미 있음"
  else
    echo "→ $f 내려받는 중…"
    curl -fL --retry 3 -o "$PREFIX/voices/$f.part" "$VOICE_URL/$f"
    mv "$PREFIX/voices/$f.part" "$PREFIX/voices/$f"   # 중간에 끊긴 파일을 남기지 않는다
    echo "✓ $f"
  fi
done

# ── 3. 동작 확인 ─────────────────────────────────────────────────────────
echo "→ 합성 테스트…"
TEST_WAV="$(mktemp -t piper-test-XXXXXX.wav)"
echo "piper is ready" | "$PREFIX/piper/piper" \
  --model "$PREFIX/voices/${VOICE}.onnx" \
  --output_file "$TEST_WAV" 2>/dev/null
test -s "$TEST_WAV" || { echo "✗ 합성 실패"; exit 1; }
echo "✓ 합성 OK ($(stat -c %s "$TEST_WAV") bytes) → $TEST_WAV"

cat <<EOF

설치 완료. voice.yaml 에 넣을 경로:

  piper_bin:   $PREFIX/piper/piper
  piper_model: $PREFIX/voices/${VOICE}.onnx

소리를 들어보려면:
  paplay "$TEST_WAV"

⚠ 헤드리스(모니터 없이 SSH) 운용에는 아래가 필요하다. 안 하면 세션이 끊길 때
  PulseAudio 가 같이 내려가서 소리가 안 난다:

  sudo loginctl enable-linger $USER
EOF
