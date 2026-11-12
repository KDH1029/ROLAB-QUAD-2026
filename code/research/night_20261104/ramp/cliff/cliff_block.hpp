// ==CLIFF_BEGIN== (ramp/cliff/test_cliff_cpp.sh 가 이 구간만 떼어 파이썬 원형 terrain_cliff.cliff_edges 와 대조한다 — std 만 쓸 것)
struct CliffStats { int edge = 0, cliff = 0, empty = 0; };

// 가장자리 칸에서 중심거리 r_min~reach 안 칸 오프셋
std::vector<std::array<int, 2>> cliff_offsets(double cell, double r_min, double reach) {
  std::vector<std::array<int, 2>> o;
  const int R = static_cast<int>(std::ceil(reach / cell + 1e-9));
  for (int i = -R; i <= R; ++i)
    for (int j = -R; j <= R; ++j) {
      const double d = cell * std::hypot(static_cast<double>(i), static_cast<double>(j));
      if (d >= r_min - 1e-9 && d <= reach + 1e-9) o.push_back({i, j});
    }
  return o;
}

// n×n 격자(k = i·n + j). 가장자리 = T 칸 중 격자 안 8-이웃에 T 아닌 칸이 있는 것.
// 낭떠러지 = 가장자리 중 바깥(오프셋) T 아닌 유효 칸의 hi 가 가장자리 hi − drop 보다 낮은 게 min_low 개 이상.
// 빈 가장자리 = 바깥 T 아닌 칸이 있는데 전부 데이터 없음(세기만).
// band > 0 이면 가장자리 hi 가 발밑 지면 평면(plane6 = fx,fy,fz,nx,ny,nz; 격자 원점 ox,oy, 칸 cell)에서 ±band 안일 때만 낭떠러지.
CliffStats find_cliff(int n, const std::vector<float> & hi, const std::vector<int> & cnt, const std::vector<uint8_t> & trav,
                      const std::vector<std::array<int, 2>> & offs, double drop, int min_low,
                      std::vector<uint8_t> & cliff, std::vector<uint8_t> * edge_out = nullptr, std::vector<uint8_t> * empty_out = nullptr,
                      double band = -1.0, const double * plane6 = nullptr, double ox = 0.0, double oy = 0.0, double cell = 0.05) {
  CliffStats st;
  cliff.assign(static_cast<size_t>(n) * n, 0);
  if (edge_out) edge_out->assign(static_cast<size_t>(n) * n, 0);
  if (empty_out) empty_out->assign(static_cast<size_t>(n) * n, 0);
  for (int i = 0; i < n; ++i)
    for (int j = 0; j < n; ++j) {
      const int k = i * n + j;
      if (!trav[k]) continue;
      bool edge = false;
      for (int a = std::max(0, i - 1); a <= std::min(n - 1, i + 1) && !edge; ++a)
        for (int c = std::max(0, j - 1); c <= std::min(n - 1, j + 1); ++c)
          if (!trav[a * n + c]) { edge = true; break; }
      if (!edge) continue;
      ++st.edge;
      if (edge_out) (*edge_out)[k] = 1;
      const double thr = static_cast<double>(hi[k]) - drop;
      int nlow = 0, nval = 0, nnon = 0;
      for (const auto & o : offs) {
        const int a = i + o[0], c = j + o[1];
        if (a < 0 || c < 0 || a >= n || c >= n) continue;
        const int kk = a * n + c;
        if (trav[kk]) continue;
        ++nnon;
        if (!cnt[kk]) continue;
        ++nval;
        if (static_cast<double>(hi[kk]) < thr) ++nlow;
      }
      bool in_band = true;
      if (band > 0 && plane6) {
        const double x = ox + (i + 0.5) * cell, y = oy + (j + 0.5) * cell, nzc = std::max(plane6[5], 0.5);
        const double zp = plane6[2] - (plane6[3] * (x - plane6[0]) + plane6[4] * (y - plane6[1])) / nzc;
        in_band = std::fabs(static_cast<double>(hi[k]) - zp) <= band;
      }
      if (nlow >= min_low && in_band) { cliff[k] = 1; ++st.cliff; }
      else if (nval == 0 && nnon > 0) { ++st.empty; if (empty_out) (*empty_out)[k] = 1; }
    }
  return st;
}
// ==CLIFF_END==
