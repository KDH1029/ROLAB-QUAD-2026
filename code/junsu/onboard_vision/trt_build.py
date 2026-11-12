#!/usr/bin/env python3
"""ONNX → TensorRT 엔진 (FP16 또는 INT8 양자화). 내부 PC(Jetson)에서 실행한다 — 엔진은 기기마다 새로 만든다.
사용:
  python3 trt_build.py tl5cone_384x640.onnx tl5cone_384x640_fp16.engine --mode fp16
  python3 trt_build.py tl5cone_384x640.onnx tl5cone_384x640_int8.engine --mode int8 --calib calib/
INT8 보정: calib/ 의 사진을 하나씩 같은 전처리(tl_yolo_common.letterbox)로 넣어 층마다 값 범위를 잰다.
보정 결과는 <엔진>.calib 에 남겨 두므로, 같은 사진으로 다시 만들 때는 보정을 건너뛴다.
★ Python 3.8 호환. 무거운 작업이라 nice 19 로 돌릴 것(jetson_vision.sh build 가 그렇게 부른다).
"""
import argparse
import glob
import json
import os
import sys
import time

import cv2
import numpy as np
import tensorrt as trt

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import tl_yolo_common as C  # noqa: E402
from trt_engine import LOGGER, cuda_malloc, h2d  # noqa: E402


def make_calibrator(algo, files, shape, cache):
    base = trt.IInt8MinMaxCalibrator if algo == 'minmax' else trt.IInt8EntropyCalibrator2

    class Calib(base):
        def __init__(self):
            base.__init__(self)
            self.files, self.i, self.t0 = files, 0, time.time()
            self.d = cuda_malloc(int(np.prod(shape)) * 4)

        def get_batch_size(self):
            return 1

        def get_batch(self, names):
            while self.i < len(self.files):
                bgr = cv2.imread(self.files[self.i])
                self.i += 1
                if bgr is None:
                    continue
                blob = C.letterbox(bgr, (shape[2], shape[3]))[0]
                h2d(self.d, np.ascontiguousarray(blob, dtype=np.float32))
                if self.i % 50 == 0 or self.i == len(self.files):
                    print('  보정 %d/%d장 (%.0f초)' % (self.i, len(self.files), time.time() - self.t0), flush=True)
                return [int(self.d.value)]
            return None

        def read_calibration_cache(self):
            if os.path.exists(cache):
                print('  보정 결과 재사용: %s' % cache, flush=True)
                with open(cache, 'rb') as f:
                    return f.read()
            return None

        def write_calibration_cache(self, data):
            with open(cache, 'wb') as f:
                f.write(data)

    return Calib()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('onnx')
    ap.add_argument('out')
    ap.add_argument('--mode', choices=['fp16', 'int8'], required=True)
    ap.add_argument('--calib', default=os.path.join(HERE, 'calib'))
    ap.add_argument('--calib-n', type=int, default=400)
    ap.add_argument('--algo', choices=['entropy', 'minmax'], default='entropy')
    ap.add_argument('--workspace-mb', type=int, default=1024)
    a = ap.parse_args()
    t0 = time.time()
    print('TensorRT %s  %s → %s (%s)' % (trt.__version__, a.onnx, a.out, a.mode), flush=True)
    builder = trt.Builder(LOGGER)
    net = builder.create_network(1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH))
    parser = trt.OnnxParser(net, LOGGER)
    with open(a.onnx, 'rb') as f:
        if not parser.parse(f.read()):
            for i in range(parser.num_errors):
                print('ONNX 오류:', parser.get_error(i))
            sys.exit(1)
    shape = tuple(net.get_input(0).shape)
    print('입력 %s  출력 %s' % (shape, tuple(net.get_output(0).shape)), flush=True)
    cfg = builder.create_builder_config()
    ws = a.workspace_mb << 20
    try:
        cfg.set_memory_pool_limit(trt.MemoryPoolType.WORKSPACE, ws)
    except AttributeError:
        cfg.max_workspace_size = ws
    if builder.platform_has_fast_fp16:
        cfg.set_flag(trt.BuilderFlag.FP16)   # INT8 이 안 되는 층은 FP16 으로 (FP32 로 떨어지지 않게)
    meta = {'mode': a.mode, 'onnx': os.path.basename(a.onnx), 'trt': trt.__version__, 'input': list(shape)}
    if a.mode == 'int8':
        if not builder.platform_has_fast_int8:
            print('★ 이 기기는 INT8 가속이 없다'); sys.exit(1)
        files = sorted(glob.glob(os.path.join(a.calib, '*.jpg')) + glob.glob(os.path.join(a.calib, '*.png')))
        if len(files) > a.calib_n:
            files = [files[i] for i in np.linspace(0, len(files) - 1, a.calib_n).astype(int)]
        if not files:
            print('★ 보정 사진 없음: %s' % a.calib); sys.exit(1)
        print('INT8 보정 사진 %d장 (%s)' % (len(files), a.algo), flush=True)
        cfg.set_flag(trt.BuilderFlag.INT8)
        cfg.int8_calibrator = make_calibrator(a.algo, files, shape, a.out + '.calib')
        meta.update(calib_n=len(files), algo=a.algo)
    ser = builder.build_serialized_network(net, cfg)
    if ser is None:
        print('★ 엔진 만들기 실패'); sys.exit(1)
    with open(a.out, 'wb') as f:
        f.write(ser)
    meta['build_s'] = round(time.time() - t0, 1)
    meta['bytes'] = os.path.getsize(a.out)
    with open(a.out + '.json', 'w') as f:
        json.dump(meta, f, ensure_ascii=False, indent=1)
    print('■ 완료 %s (%.1f MB, %.0f초)' % (a.out, meta['bytes'] / 1e6, meta['build_s']), flush=True)


if __name__ == '__main__':
    main()
