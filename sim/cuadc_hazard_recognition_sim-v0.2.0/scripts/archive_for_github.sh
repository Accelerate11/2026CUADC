#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
version="$(sed -n 's:.*<version>\(.*\)</version>.*:\1:p' "$repo_root/package.xml" | head -n1)"
output_dir="${1:-$repo_root/dist}"
archive_name="cuadc_hazard_recognition_sim-v${version}"

mkdir -p "$output_dir"
tar --exclude='.git' --exclude='build' --exclude='install' --exclude='log' \
  --exclude='dist' --exclude='cuadc_outputs' --exclude='__pycache__' \
  --exclude='*.onnx' -czf "$output_dir/$archive_name.tar.gz" \
  -C "$(dirname "$repo_root")" "$(basename "$repo_root")"

if command -v zip >/dev/null 2>&1; then
  (cd "$(dirname "$repo_root")" && zip -qr "$output_dir/$archive_name.zip" \
    "$(basename "$repo_root")" -x '*/.git/*' '*/build/*' '*/install/*' \
    '*/log/*' '*/dist/*' '*/cuadc_outputs/*' '*/__pycache__/*' '*.onnx')
fi

(cd "$output_dir" && sha256sum "$archive_name.tar.gz" > SHA256SUMS && \
  { [[ ! -f "$archive_name.zip" ]] || sha256sum "$archive_name.zip" >> SHA256SUMS; })
cat "$output_dir/SHA256SUMS"
