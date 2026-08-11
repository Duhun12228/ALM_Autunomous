#!/usr/bin/env bash
# 음성 안내를 부팅 즉시 자동 실행시킨다 (systemd 사용자 서비스).
#
#   scripts/install_voice_service.sh            설치 + 즉시 기동
#   scripts/install_voice_service.sh --uninstall 제거
#
# 왜 사용자 서비스인가: 오디오가 PulseAudio 를 타는데 그것이 사용자 세션
# 프로세스라서, root 로 띄우면 소리가 안 난다.
set -euo pipefail

UNIT="alm-voice.service"
SRC="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/${UNIT}"
DEST_DIR="${HOME}/.config/systemd/user"

if [[ "${1:-}" == "--uninstall" ]]; then
    systemctl --user disable --now "${UNIT}" 2>/dev/null || true
    rm -f "${DEST_DIR}/${UNIT}"
    systemctl --user daemon-reload
    echo "제거했습니다."
    exit 0
fi

# 워크스페이스가 빌드돼 있어야 한다. 없는 채로 enable 하면 5초마다 재시작을
# 반복하며 저널만 채운다.
SETUP="${HOME}/ALM_Autunomous/ALM_auto_ws/install/setup.bash"
if [[ ! -f "${SETUP}" ]]; then
    echo "✗ 워크스페이스가 빌드되지 않았습니다: ${SETUP}" >&2
    echo "  먼저: cd ~/ALM_Autunomous/ALM_auto_ws && colcon build --symlink-install" >&2
    exit 1
fi

mkdir -p "${DEST_DIR}"
install -m 644 "${SRC}" "${DEST_DIR}/${UNIT}"
systemctl --user daemon-reload
systemctl --user enable --now "${UNIT}"

sleep 3
systemctl --user --no-pager --lines=15 status "${UNIT}" || true

echo
echo "── 확인 ────────────────────────────────────────────────"
echo "  상태:  systemctl --user status ${UNIT}"
echo "  로그:  journalctl --user -u ${UNIT} -f"
echo "  중지:  systemctl --user stop ${UNIT}"
echo

# 여기가 이 스크립트에서 제일 중요한 부분이다. linger 가 꺼져 있으면 사용자
# 세션이 끝날 때(=SSH 종료, 모니터 분리) 서비스도 같이 내려간다. 실차는 로그인
# 세션이 아예 없는 상태로 도는 것이 정상이므로, 이게 없으면 소리가 안 난다.
if [[ "$(loginctl show-user "${USER}" -p Linger --value 2>/dev/null)" != "yes" ]]; then
    cat <<EOF
⚠ linger 가 꺼져 있습니다 (Linger=no).

  지금 상태로는 로그인 세션이 끝나면 — SSH 를 끊거나 모니터를 떼면 —
  이 서비스도 함께 종료됩니다. 부팅만 하고 아무도 로그인하지 않는
  실차 운용에서는 아예 시작조차 하지 않습니다.

  아래를 실행하세요 (sudo 필요, 1회):

      sudo loginctl enable-linger ${USER}

EOF
else
    echo "✓ linger 켜져 있음 — 로그인 없이도 부팅 시 자동 기동합니다."
fi
