#!/usr/bin/env bash
# Sequentially runs fast_lio on all 37 per-segment bags, one fresh process
# each (no timestamp jumps within any single run).
BAGROOT=~/ws/junsu/maps/segments/bags
OUTROOT=~/ws/junsu/maps/segments/pcd
CFG=~/ws/junsu/config/go2_xt16_mapping_rgosu.yaml
SCRIPT=~/ws/junsu/tools/run_fastlio_segment.sh

mkdir -p "$OUTROOT"

for d in "$BAGROOT"/*/; do
  name=$(basename "$d")
  echo "=== [$(date +%T)] starting segment: $name ==="
  bash "$SCRIPT" "$d" "$CFG" "$name" "$OUTROOT/${name}.pcd"
  echo "=== [$(date +%T)] finished segment: $name, pcd exists: $(test -f "$OUTROOT/${name}.pcd" && echo yes || echo NO) ==="
done
echo "ALL SEGMENTS DONE"
