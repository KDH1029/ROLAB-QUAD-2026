"""OpenVINO IR(FP16)에서 꺼낸 합성곱 가중치(BN 합쳐진 상태)로 YOLOv8n(nc=4)을 되살린다."""
import sys, numpy as np, torch, torch.nn as nn
from ultralytics.nn.tasks import DetectionModel
W = dict(np.load(sys.argv[1]))
m = DetectionModel('yolov8n.yaml', nc=80, verbose=False)   # 머리 폭 c3=80 은 nc=80 으로 만든 모델에서 온 것
det = m.model[-1]
for i in range(3): det.cv3[i][2] = nn.Conv2d(det.cv3[i][2].in_channels, 4, 1)
det.nc = 4; det.no = 4 + det.reg_max * 4
m.nc = 4; m.names = {0: 'tl_red', 1: 'tl_green', 2: 'unknown', 3: 'cone'}
m.eval(); m.fuse(verbose=False)
convs = {n: c for n, c in m.named_modules() if isinstance(c, nn.Conv2d)}
miss, used = [], set()
for n, c in convs.items():
    if n.endswith('dfl.conv'): continue
    wk = n + '.weight'
    if wk not in W: miss.append(n); continue
    assert tuple(W[wk].shape) == tuple(c.weight.shape), (n, W[wk].shape, c.weight.shape)
    c.weight.data = torch.from_numpy(W[wk]); used.add(wk)
    bk = n + '.bias'
    if c.bias is None: c.bias = nn.Parameter(torch.zeros(c.out_channels))
    c.bias.data = torch.from_numpy(W[bk]); used.add(bk)
left = [k for k in W if k not in used and 'dfl' not in k]
print('torch convs', len(convs), 'missing', miss, 'unused', left)
assert not miss and not left
torch.save({'model': m, 'names': m.names}, sys.argv[2])
print('saved', sys.argv[2])
