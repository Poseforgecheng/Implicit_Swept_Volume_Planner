#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
VENV_DIR="${IRSIM_VENV_DIR:-${PACKAGE_DIR}/.venv}"

version_is_supported() {
    "$1" -c 'import sys; raise SystemExit(0 if sys.version_info >= (3, 10) else 1)' \
        >/dev/null 2>&1
}

if [[ -n "${IRSIM_BOOTSTRAP_PYTHON:-}" ]]; then
    BOOTSTRAP_PYTHON="${IRSIM_BOOTSTRAP_PYTHON}"
elif version_is_supported python3; then
    BOOTSTRAP_PYTHON="$(command -v python3)"
elif [[ -x "${HOME}/miniconda3/bin/python3" ]] && \
     version_is_supported "${HOME}/miniconda3/bin/python3"; then
    BOOTSTRAP_PYTHON="${HOME}/miniconda3/bin/python3"
else
    echo "ERROR: IR-SIM 2.10.2 requires Python >= 3.10." >&2
    echo "Set IRSIM_BOOTSTRAP_PYTHON=/path/to/python3.10+ and run again." >&2
    exit 2
fi

echo "Creating IR-SIM environment with: ${BOOTSTRAP_PYTHON}"
echo "Virtual environment: ${VENV_DIR}"
"${BOOTSTRAP_PYTHON}" -m venv "${VENV_DIR}"
"${VENV_DIR}/bin/python3" -m pip install --upgrade pip setuptools wheel
"${VENV_DIR}/bin/python3" -m pip install \
    "ir-sim==2.10.2" \
    "rospkg==1.6.0" \
    "catkin-pkg==1.1.0"

PYTHONPATH="/opt/ros/noetic/lib/python3/dist-packages${PYTHONPATH:+:${PYTHONPATH}}" \
    "${VENV_DIR}/bin/python3" -c \
    'import irsim, rospy; from nav_msgs.msg import OccupancyGrid; print("IR-SIM/ROS Python bridge imports: OK")'

echo
echo "Setup complete. The default launch file uses:"
echo "  ${VENV_DIR}/bin/python3"
