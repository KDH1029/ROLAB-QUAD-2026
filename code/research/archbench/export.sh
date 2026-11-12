#!/usr/bin/env bash
# 발표 자료 HTML 조각 → 단독 HTML + A4 가로 PDF
set -e
SRC=${SRC:-./labmeeting/go2_basetech.html}
OUT=~/ws/docs/labmeeting; NAME=Go2_기반기술_정리_20261008
python3 - "$SRC" "$OUT/$NAME.html" <<'PY'
import sys
t=open(sys.argv[1],encoding='utf-8').read()
i=t.index('</style>')+len('</style>')
head,body=t[:i],t[i:]
doc=('<!doctype html>\n<html lang="ko">\n<head>\n<meta charset="utf-8">\n'
     '<meta name="viewport" content="width=device-width, initial-scale=1">\n'+head+'\n</head>\n<body>\n'+body+'\n</body>\n</html>\n')
open(sys.argv[2],'w',encoding='utf-8').write(doc)
PY
google-chrome --headless=new --disable-gpu --no-pdf-header-footer --virtual-time-budget=8000 \
  --print-to-pdf="$OUT/$NAME.pdf" "file://$OUT/$NAME.html" 2>/dev/null
ls -la "$OUT"
