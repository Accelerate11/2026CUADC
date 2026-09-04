#!/usr/bin/env bash
set -eo pipefail

src_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
archive="${HOME}/cuadc_rescue_sim_github_archive.tar.gz"

tar \
  --exclude='build' \
  --exclude='install' \
  --exclude='log' \
  --exclude='.git' \
  --exclude='__pycache__' \
  -czf "$archive" \
  -C "$(dirname "$src_dir")" \
  "$(basename "$src_dir")"

echo "Wrote $archive"
