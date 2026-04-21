#!/usr/bin/env bash
set -euo pipefail

BASE_URL="https://f005.backblazeb2.com/file/inchor-maps/autoware"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

FILES=(
  "map/AA_Fixed_V14/pointcloud_map.pcd"
  "map/AA_Fixed_V14/AA_Fixed_V14_lanelet2.osm"
  "map/AA_Fixed_V14/map_projector_info.yaml"
)

updated=0
skipped=0

md5_of_file() {
  if command -v md5sum &>/dev/null; then
    md5sum "$1" | awk '{print $1}'
  else
    md5 -q "$1"
  fi
}

for rel_path in "${FILES[@]}"; do
  local_file="$REPO_ROOT/$rel_path"
  remote_url="$BASE_URL/$rel_path"
  local_dir="$(dirname "$local_file")"

  # Fetch remote hash (tiny file, fast)
  remote_hash=$(curl -fsSL "${remote_url}.md5" 2>/dev/null || echo "")

  if [ -z "$remote_hash" ]; then
    echo "[WARN] Could not fetch hash for $rel_path — skipping"
    continue
  fi

  # Compare to local hash if file exists
  if [ -f "$local_file" ]; then
    local_hash=$(md5_of_file "$local_file")
    if [ "$local_hash" = "$remote_hash" ]; then
      echo "[SKIP] $rel_path (up to date)"
      ((skipped++)) || true
      continue
    fi
  fi

  # Download
  mkdir -p "$local_dir"
  echo "[DOWN] $rel_path"
  curl -fL --progress-bar "$remote_url" -o "$local_file"
  ((updated++)) || true
done

echo ""
echo "Done. $updated updated, $skipped skipped."
