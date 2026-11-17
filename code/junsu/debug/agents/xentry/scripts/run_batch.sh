#!/usr/bin/env bash
# xentry: 도메인 78 에서 한 번에 한 판씩 차례로. 사용: run_batch.sh <접두> <경로파일> <판 수> (조건은 환경변수)
P=$1; ROUTE=$2; N=$3; X=$HOME/ws/junsu_v2/debug/agents/xentry
for i in $(seq 1 $N); do
  SEED=$i $X/scripts/run_rec.sh ${P}_$i $ROUTE > $X/rec/${P}_$i.out 2>&1
  [ $i -gt 1 ] && rm -f $X/rec/${P}_$i.jsonl   # 큰 기록은 첫 판만 남김
  tail -2 $X/rec/${P}_$i.out
done
