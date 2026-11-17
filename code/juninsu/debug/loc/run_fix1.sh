#!/usr/bin/env bash
# AMCL 문제 재현·해결 시험 묶음(4개씩, 한 묶음 끝나야 다음). 출력 runf_*.txt
cd "$(dirname "$(readlink -f "$0")")"; T45="-0.025,-0.039,-0.0604"; B=$PWD/out/r1645
run_batch(){ local d=78 pids=(); while read -r sc mv scan bag tag; do
    MV=$([ "$mv" = "-" ] || echo $mv) SCAN=$scan BANDS=$B/$bag ./amcl_run.sh r1645 $sc tuned $d "$T45" > runf_$tag.txt 2>&1 & pids+=($!); d=$((d+1)); done; wait "${pids[@]}"; }
run_batch <<'L'
S0 A /scan_D bands D_A_S0
S0 A /scan_Drs scan_rs Drs_A_S0
S0 A /scan_rs scan_rs rs_A_S0
S0 - /scan_Drs scan_rs Drs_S0
L
echo "묶음1 끝 $(date +%T)"
run_batch <<'L'
S1 A /scan_D bands D_A_S1
S1 A /scan_Drs scan_rs Drs_A_S1
S1 A /scan_rs scan_rs rs_A_S1
S1 - /scan_Drs scan_rs Drs_S1
L
echo "묶음2 끝 $(date +%T)"
