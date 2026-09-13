#!/usr/bin/env bash

set -Eeuo pipefail
PACKAGE_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_NAME="$(basename -- "${PACKAGE_ROOT}")"
VERSION="$(tr -d '[:space:]' < "${PACKAGE_ROOT}/VERSION")"
OUTPUT_DIR="${1:-${PACKAGE_ROOT}/dist}"
ARCHIVE="${OUTPUT_DIR}/${PACKAGE_NAME}-${VERSION}.tar.gz"

mkdir -p -- "${OUTPUT_DIR}"
tar \
  --exclude="${PACKAGE_NAME}/dist" \
  --exclude='*/__pycache__' \
  --exclude='*.pyc' \
  -C "$(dirname -- "${PACKAGE_ROOT}")" \
  -czf "${ARCHIVE}" \
  "${PACKAGE_NAME}"

sha256sum "${ARCHIVE}"
echo "迁移包已生成：${ARCHIVE}"

