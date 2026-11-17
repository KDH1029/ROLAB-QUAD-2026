// C++ find_cliff (terrain_veto_node.cpp 의 ==CLIFF_BEGIN==~==CLIFF_END== 구간 그대로) vs 파이썬 원형 결과(synth_grid_*.txt) 대조
// 입력 줄: hi(−999=빈칸) T cliff edge empty  (n×n, k = i·n + j). 인자: 파일 n cell drop reach rmin min_low
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <vector>
#include <chrono>
#include "cliff_block.hpp"
int main(int argc, char ** argv) {
  if (argc < 8) { std::fprintf(stderr, "사용: %s grid.txt n cell drop reach rmin min_low\n", argv[0]); return 2; }
  const int n = std::atoi(argv[2]); const double cell = std::atof(argv[3]), drop = std::atof(argv[4]), reach = std::atof(argv[5]), rmin = std::atof(argv[6]);
  const int min_low = std::atoi(argv[7]);
  FILE * f = std::fopen(argv[1], "r"); if (!f) return 3;
  std::vector<float> hi(n * n); std::vector<int> cnt(n * n); std::vector<uint8_t> T(n * n), pc(n * n), pe(n * n), pm(n * n);
  for (int k = 0; k < n * n; ++k) {
    double h; int t, c, e, m;
    if (std::fscanf(f, "%lf %d %d %d %d", &h, &t, &c, &e, &m) != 5) { std::fprintf(stderr, "읽기 실패 %d\n", k); return 4; }
    cnt[k] = h > -900; hi[k] = cnt[k] ? static_cast<float>(h) : -INFINITY; T[k] = t; pc[k] = c; pe[k] = e; pm[k] = m;
  }
  std::fclose(f);
  auto offs = cliff_offsets(cell, rmin, reach);
  std::vector<uint8_t> cl, ed, em;
  double band = -1.0, pl[6] = {0, 0, 0, 0, 0, 1}, ox = 0, oy = 0;   // 선택 인자: band fx fy fz nx ny nz ox oy
  if (argc >= 17) { band = std::atof(argv[8]); for (int i = 0; i < 6; ++i) pl[i] = std::atof(argv[9 + i]); ox = std::atof(argv[15]); oy = std::atof(argv[16]); }
  CliffStats st = find_cliff(n, hi, cnt, T, offs, drop, min_low, cl, &ed, &em, band, pl, ox, oy, cell);
  if (std::getenv("BENCH")) {   // 시간 재기: find_cliff 만 200회
    const auto t0 = std::chrono::steady_clock::now();
    for (int r = 0; r < 200; ++r) find_cliff(n, hi, cnt, T, offs, drop, min_low, cl, nullptr, nullptr, band, pl, ox, oy, cell);
    std::printf("find_cliff 평균 %.3f ms\n", std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count() / 200);
  }
  int dc = 0, de = 0, dm = 0, pyc = 0, pye = 0, pym = 0;
  for (int k = 0; k < n * n; ++k) { dc += cl[k] != pc[k]; de += ed[k] != pe[k]; dm += em[k] != pm[k]; pyc += pc[k]; pye += pe[k]; pym += pm[k]; }
  std::printf("%s  오프셋 %zu | C++ 가장자리 %d 낭떠러지 %d 빈 %d | 파이썬 %d %d %d | 불일치 칸: 낭떠러지 %d 가장자리 %d 빈 %d  → %s\n",
              argv[1], offs.size(), st.edge, st.cliff, st.empty, pye, pyc, pym, dc, de, dm, (dc + de + dm) ? "불일치" : "일치");
  return (dc + de + dm) ? 1 : 0;
}
