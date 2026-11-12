#!/usr/bin/env bash
# gosu 도구를 젯슨에 올린다. **검사를 통과해야만** 올라간다.
#
# 【왜 이 파일이 있나 — 2026-09-13】
# `tyaw` 를 정의하지 않고 쓴 코드를 그대로 올려서, 현장에서 정밀 접근이
# "name 'tyaw' is not defined" 로 통째로 건너뛰어졌다. 문법 검사(ast.parse)는
# 그걸 못 잡는다 — 문법은 맞기 때문이다. 이름이 있는지까지 보는 검사가 필요하다.
set -u
SRC=~/ws/lab_visual/tools
HOST=go2-jetson
DST='~/gosu/lab_visual/tools'
C0=$'\033[0m'; CR=$'\033[1;31m'; CG=$'\033[1;32m'; CY=$'\033[1;33m'

files=("$@")
[ ${#files[@]} -eq 0 ] && files=("$SRC"/*.py)

bad=0
for f in "${files[@]}"; do
    b=$(basename "$f")
    if ! python3 -c "import ast,io,sys; ast.parse(io.open('$SRC/$b',encoding='utf-8').read())" 2>/dev/null; then
        printf "${CR}✘ %s 문법 오류${C0}\n" "$b"; bad=1; continue
    fi
    out=$(python3 -m pyflakes "$SRC/$b" 2>&1 \
          | grep -viE "imported but unused|assigned to but never used|unable to detect undefined names")
    if [ -n "$out" ]; then
        printf "${CR}✘ %s${C0}\n" "$b"; echo "$out" | sed 's/^/    /'; bad=1
    fi
done
if [ "$bad" = 1 ]; then
    printf "\n${CR}★ 검사에 걸린 파일이 있다 — 아무것도 올리지 않았다${C0}\n"
    exit 1
fi

for f in "${files[@]}"; do
    b=$(basename "$f")
    scp -q "$SRC/$b" "$HOST:/tmp/$b" || { printf "${CR}✘ %s 전송 실패${C0}\n" "$b"; exit 1; }
    ssh "$HOST" "cp /tmp/$b $DST/$b" || exit 1
    printf "${CG}✔${C0} %s\n" "$b"
done
printf "${CG}전부 올렸다${C0}\n"
