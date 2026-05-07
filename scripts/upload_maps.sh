#!/usr/bin/env bash
set -euo pipefail

BUCKET="b2:inchor-maps/autoware"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
TMP_DIR=$(mktemp -d)
trap 'rm -rf "$TMP_DIR"' EXIT

FILES=(
  "map/AA_Fixed_V14/pointcloud_map.pcd"
  "map/AA_Fixed_V14/AA_Fixed_V14_lanelet2.osm"
  "map/AA_Fixed_V14/map_projector_info.yaml"
  "map/520toRapp/pointcloud_map.pcd"
  "map/520toRapp/520toRapp_lanelet2.osm"
  "map/520toRapp/map_projector_info.yaml"
)

for rel_path in "${FILES[@]}"; do
  local_file="$REPO_ROOT/$rel_path"
  remote_path="$BUCKET/$rel_path"
  tmp_md5="$TMP_DIR/$(echo "$rel_path" | tr '/' '_').md5"

  if [ ! -f "$local_file" ]; then
    echo "[SKIP] $rel_path (not found locally)"
    continue
  fi

  echo "[HASH] $rel_path"
  if command -v md5sum &>/dev/null; then
    hash=$(md5sum "$local_file" | awk '{print $1}')
  else
    hash=$(md5 -q "$local_file")
  fi
  echo "$hash" > "$tmp_md5"

  echo "[UPLOAD] $rel_path"
  rclone copyto "$local_file" "$remote_path"
  rclone copyto "$tmp_md5" "${remote_path}.md5"
  echo "[DONE] $rel_path -> $remote_path"
done

echo ""
echo "Upload complete."
