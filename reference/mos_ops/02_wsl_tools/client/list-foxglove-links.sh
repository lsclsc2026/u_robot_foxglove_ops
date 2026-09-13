#!/usr/bin/env bash

set -Eeuo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
CONFIG_FILE="${1:-${SCRIPT_DIR}/robots.conf}"

FOXGLOVE_LAYOUT_ID=""
if [[ -r "${SCRIPT_DIR}/fleet.env" ]]; then
  # shellcheck disable=SC1091
  source "${SCRIPT_DIR}/fleet.env"
fi

while read -r name host local_port extra; do
  [[ -n "${name:-}" && "${name}" != \#* ]] || continue
  url="https://app.foxglove.dev/~/view?ds=foxglove-websocket&ds.url=ws%3A%2F%2Flocalhost%3A${local_port}"
  if [[ -n "${FOXGLOVE_LAYOUT_ID}" ]]; then
    url+="&layoutId=${FOXGLOVE_LAYOUT_ID}"
  fi
  printf '%-12s %s\n' "${name}" "${url}"
done < "${CONFIG_FILE}"

