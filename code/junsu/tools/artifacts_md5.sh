#!/usr/bin/env bash
# git 밖 큰 파일(모델·엔진·지도)의 md5 를 ARTIFACTS.md5 로 — 커밋 전에 돌리면 무엇이 바뀌었는지 git 이 보여 준다.
cd "$(dirname "$(readlink -f "$0")")/.." || exit 1
{
  echo "# 추적하지 않는 큰 파일 md5 (tools/artifacts_md5.sh 로 다시 만듦). 내부 PC 엔진은 ~/junsu_vision 에 있다"
  find onboard_vision -maxdepth 1 \( -name '*.onnx' -o -name '*.pt' \) -print0 | sort -z | xargs -0 md5sum
  find onboard_vision/models_1104 \( -name '*.bin' -o -name '*.xml' \) -print0 | sort -z | xargs -0 md5sum
  find maps -name '*.pcd' -not -path 'maps/divergence_test/*' -print0 | sort -z | xargs -0 md5sum
} > ARTIFACTS.md5
echo "ARTIFACTS.md5 $(grep -vc '^#' ARTIFACTS.md5)개"
