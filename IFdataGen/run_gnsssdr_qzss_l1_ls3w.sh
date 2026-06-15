#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
GNSS_SDR="${SCRIPT_DIR}/../../gnss-sdr/build/src/main/gnss-sdr"
CONFIG="${SCRIPT_DIR}/gnsssdr-configs/QZSS_L1CA_LS3W_10p5.conf"
LOG="${SCRIPT_DIR}/gnsssdr_qzss_l1_ls3w_10p5.log"
LS3W="${SCRIPT_DIR}/GPS_QZSS_L1CA_10p5m_180s_1bit.LS3W"
INI="${SCRIPT_DIR}/GPS_QZSS_L1CA_10p5m_180s_1bit.ini"

if [[ ! -x "${GNSS_SDR}" ]]; then
  echo "GNSS-SDR binary not found or not executable: ${GNSS_SDR}" >&2
  exit 1
fi

if [[ ! -f "${LS3W}" ]]; then
  echo "LS3W file not found: ${LS3W}" >&2
  exit 1
fi

if [[ ! -f "${INI}" ]]; then
  echo "LabSat ini file not found: ${INI}" >&2
  exit 1
fi

cd "${SCRIPT_DIR}"
echo "Running GNSS-SDR QZSS L1 C/A LS3W analysis"
echo "  config: ${CONFIG}"
echo "  input : ${LS3W}"
echo "  log   : ${LOG}"

"${GNSS_SDR}" --config_file="${CONFIG}" 2>&1 | tee "${LOG}"
