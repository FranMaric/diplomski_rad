#!/bin/bash
set -euo pipefail

for episode_dir in line_bags/*/; do
    echo "Processing ${episode_dir}"
    mcap_file=$(ls "${episode_dir}"*.mcap 2>/dev/null | head -1)
    stem=$(basename "${mcap_file}" .mcap)
    out_dir="${episode_dir}${stem}_30hz"
    rm -rf "${out_dir}"
    python3 resample_30hz.py "${episode_dir}"
done