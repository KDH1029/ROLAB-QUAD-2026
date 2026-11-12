import sys, torch
m = torch.load(sys.argv[1], weights_only=False)['model'].float().eval()
det = m.model[-1]; det.export = True; det.format = 'onnx'; det.dynamic = False
x = torch.zeros(1, 3, 384, 640)
with torch.no_grad(): y = m(x); y = y[0] if isinstance(y, (tuple, list)) else y
print('out', tuple(y.shape))
torch.onnx.export(m, x, sys.argv[2], opset_version=17, input_names=['images'], output_names=['output0'], dynamo=False, do_constant_folding=True)
print('saved')
