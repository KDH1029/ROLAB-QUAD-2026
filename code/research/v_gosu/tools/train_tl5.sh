#!/usr/bin/env bash
# 신호등 YOLOv8n 5클래스 재학습 (NUC CPU, 최하 우선순위). 원래 가중치(tl5_aug.pt)를 못 찾아서 다시 학습한다.
# 데이터: ~/ws/datasets/tl5/ds_aug (학습 1,242 / 검증 103, 당시와 같은 검증셋)
# 시작점: 공식 COCO 사전학습 yolov8n.pt (11-01 github.com/ultralytics/assets 에서 받음)
# 사용: [DATA=데이터.yaml NAME=이름] train_tl5.sh [해상도=640] [에폭=60]
#   6클래스(신호등+콘): DATA=~/ws/datasets/tl5/ds_tl5_cone/data.yaml NAME=tl5cone_640 train_tl5.sh 640 80
SZ=${1:-640}; EP=${2:-60}
export TL5_DATA=${DATA:-$HOME/ws/datasets/tl5/ds_aug/data_nuc.yaml} TL5_NAME=${NAME:-tl5_${SZ}_aug}
export YOLO_AUTOINSTALL=false YOLO_OFFLINE=1
cd ~/ws/datasets/tl5
exec nice -n 19 ~/venv/go2/bin/python - "$SZ" "$EP" <<'PY'
import sys, os
from ultralytics import YOLO
sz, ep = int(sys.argv[1]), int(sys.argv[2])
m = YOLO('/home/ilovehan9283/ws/datasets/tl5/weights/yolov8n.pt')
m.train(data=os.environ['TL5_DATA'], imgsz=sz, epochs=ep, patience=15,
        batch=16, workers=4, cache='ram', device='cpu', amp=False, plots=True,
        project='/home/ilovehan9283/ws/datasets/tl5/runs', name=os.environ['TL5_NAME'], exist_ok=True)
print('끝 — best:', m.trainer.best)
PY
