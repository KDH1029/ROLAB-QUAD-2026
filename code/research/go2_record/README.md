# Go2 자동 녹화 (~/go2_record)

```
만든 날   2026-11-04 (로보월드 연습일)
위치      ~/go2_record  — 명령은 ~/bin 에 연결돼 있어 어느 폴더에서든 이름만 치면 된다
입력      없음 (내부 PC ssh 키가 없으면 처음 한 번 비밀번호만)
```

## 쓰는 법
```bash
go2_record_auto.sh arena3 "경기장 주행, 시작·끝에 좌우 비틀기 2초"     # 이름·메모는 생략 가능
# … 주행 …  끝내기: Ctrl+C  (NUC 녹화 마무리·점검. ★11-05: 카메라 영상은 내부 PC 에 두고 나중에 go2_record_fetch.sh all)
go2_record_check.py ~/bags/gosu_<시각>_<이름>                                  # 확인 그림: 컬러·깊이·적외선 + XT16 겹침 → 녹화폴더/check_50.png
go2_ssh.sh                                                              # 내부 PC 접속(녹화가 연 연결 재사용 — 비밀번호 안 물음). 메뉴에서 1 = foxy
```
비밀번호를 아예 안 묻게(한 번만, 내부 PC 에 키 등록 — 직접 실행): `ssh-copy-id unitree@192.168.123.18`

## 하는 일 (묻지 않고 순서대로)
1. 로봇 링크·내부 PC 응답을 최대 2분 기다림 → 로봇 본체·내부 PC·XT16·링크 속도·디스크·수신 버퍼 점검
2. 내부 PC ssh (3시간 유지)
3. 내부 PC 시계 − NUC 시계 측정(5번 중 가장 짧은 왕복)
4. D435i: USB 인식 최대 1분 기다림 → 켜기(일반 → 하드웨어 초기화 → 다른 DDS 설정 순으로 재시도) → **내부 PC 안에서 C++ 시험 녹화로 데이터 확인**
5. 내부 PC 에 카메라 토픽 녹화 시작(컬러·깊이·적외선 2 + camera_info·metadata·extrinsics·tf, 848×480 30 fps) — 실패하면 3번까지
6. XT16 드라이버 — 18 Hz 미만이면 깨끗이 다시 켬, 2번까지
7. 시각 측정기(2초마다 20번 왕복, 가장 빠른 것 — 정밀도 약 0.1 ms) + **감시자(15초마다)**: 카메라 노드·내부 PC 녹화기·XT16 드라이버·시각 측정기가 죽으면 다시 켬(카메라 녹화는 새 조각 pN 으로 이어 감)
8. NUC 녹화: 카메라를 뺀 전 토픽(mcap, zstd, 5분 조각) — NUC 에 정의 없는 형식 7개만 뺌(녹화기 멈춤 방지)
9. Ctrl+C → 내부 PC 녹화 마무리 → (★11-05 기본) 영상은 내부 PC 에 두고 대기 목록 `pending_camera.tsv` 에 적음 → 점검 → **문제와 해결 명령 요약**
   - 나중에 한 번에: `go2_record_fetch.sh` (목록·크기) · `go2_record_fetch.sh all` (전부 옮김 + NUC 시계로 보정, 끊겨도 이어 받음) · `go2_record_fetch.sh <이름일부>`
   - 예전처럼 끝날 때 바로 옮기려면 `TRANSFER=1 go2_record_auto.sh 이름`

## 결과
```
~/bags/gosu_<시각>_<이름>/                     NUC 녹화(XT16·L1·IMU·다리오도·lowstate·명령·앞머리 카메라 …)
   meta.txt                             조건 전부(시각 모델, 주기, 토픽 목록, bag 정보)
   problems.txt                         ■ 해결 못 한 문제(원인·해결 명령)   □ 자동 복구한 것
   clock_probe.csv                      내부 PC − NUC 시각 측정 전부
   camera_robotpc_p1/                   내부 PC 원본 카메라 bag(내부 PC 시계, sqlite3)
   camera_nuctime_p1/                   ★ NUC 시계로 고친 카메라 bag(mcap) — 라이다와 같이 쓰는 것
~/go2_record/logs/auto_<시각>/           실행 로그
내부 PC ~/bags/cam_<시각>_p1/            원본(옮긴 뒤 확인하고 지울 것: go2_ssh.sh 'du -sh ~/bags/*')
```
두 bag 같이 재생: `ros2 bag play ~/bags/gosu_<시각> ~/bags/gosu_<시각>/camera_nuctime_p1` 이 안 되면 `ros2 bag convert` 로 합칠 것.

## 문제와 해결 (11-04 실제로 겪은 것)
| 증상 | 원인 | 해결 |
|---|---|---|
| `No such file or directory` | 경로 | `go2_record_auto.sh` 는 ~/bin 에 연결돼 있다. 안 되면 `~/go2_record/go2_record_auto.sh` |
| 로봇 본체·내부 PC 무응답 | 전원·랜 | `ip -br link` (enp86s0 가 UP, 1000) · `ping 192.168.123.18` · 내부 PC 는 켠 뒤 1분 걸림 |
| `D435i USB 인식 안 됨` | USB 빠짐·접촉 | `go2_ssh.sh 'lsusb \| grep 8086'` → `8086:0b3a Intel RealSense 435i` 가 나와야. USB3(파란) 포트에 다시 꽂기 |
| `카메라 노드가 데이터를 안 냄` | 펌웨어/USB 전원, 다른 프로그램 점유 | `go2_ssh.sh 'tail -30 /tmp/go2_rs_*.log'` · `go2_ssh.sh 'pgrep -af realsense'` · USB 뽑았다 꽂기 |
| `XT16 18 Hz 미만` | 드라이버 중복·버퍼·전원 | `pgrep -a hesai_ros_drive`(1개) · `sysctl net.core.rmem_max`(≥26214400) · `ping 192.168.123.20` |
| `로봇 링크 100 Mb/s` | 랜 케이블 불량 | 다른 케이블 · `cat /sys/class/net/enp86s0/speed` |
| 비밀번호를 계속 물음 | ssh 연결 공유 끊김 | `rm ~/.ssh/go2_cm_*` 후 다시 · 아예 없애려면 `ssh-copy-id unitree@192.168.123.18` |
| `metadata.yaml 없음` | 녹화기 비정상 종료 | `ros2 bag reindex <폴더> sqlite3` |

## 알아 둘 것
- 내부 PC(Foxy)에서는 **파이썬 `ros2 topic hz/echo` 가 같은 PC 의 데이터를 못 받는다**(원인 미확정). 녹화기(C++)는 정상 — 그래서 확인을 C++ 시험 녹화로 한다. NUC 에서는 파이썬도 정상.
- 로그인 메뉴(`ros:foxy(1) noetic(2) ?`)에 '1' 을 자동으로 먹이는 방식은 입력이 잘려 실패한다 → 메뉴가 하는 설정(`~/.bashrc` 125~129행)을 그대로 넣어 비대화형으로 실행.
- 내부 PC 시계는 1970 에서 시작하고 약 35~40 ppm 흐른다(6분에 15 ms) → 녹화 내내 재서 보정한다. 보정 잔차 0.01 ms(11-04 실측).
- ★ 11-04 확인 그림에서 XT16 점과 깊이가 58 cm 어긋남 — 카메라 장착이 10-30 보정(아래 34°)과 달라진 듯. **걸으며 벽이 많이 보이는 녹화로 외부파라미터 재보정 필요** (`calib/d435i_to_body.json` 은 10-30 값).
