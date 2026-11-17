#!/usr/bin/env python3
"""TensorRT 엔진 실행 — torch·pycuda 없이 JetPack 기본(tensorrt 파이썬 + libcudart)만 쓴다.

내부 PC(Jetson Orin NX, JetPack 5 = TensorRT 8.5, Python 3.8)용. CUDA 메모리는 ctypes 로
libcudart 를 직접 부른다(pycuda 설치가 필요 없게). 엔진 입출력은 1개씩(images → output0)이라고 본다.
★ Python 3.8 호환으로 쓸 것.
"""
import ctypes
import ctypes.util
import os

import numpy as np

if not hasattr(np, 'bool'):   # ★ 11-05: TensorRT 8.5 의 trt.nptype 이 np.bool 을 쓴다 — 내부 PC numpy 1.24 에선 없어져 엔진 로드가 죽음
    np.bool = bool
import tensorrt as trt  # noqa: E402

H2D, D2H = 1, 2


def _load_cudart():
    cands = [ctypes.util.find_library('cudart'), 'libcudart.so', '/usr/local/cuda/lib64/libcudart.so']
    cands += sorted(p for d in ('/usr/local/cuda/lib64', '/usr/lib/aarch64-linux-gnu')
                    if os.path.isdir(d) for p in [os.path.join(d, f) for f in os.listdir(d)] if 'libcudart.so' in p)
    for c in cands:
        if not c:
            continue
        try:
            return ctypes.CDLL(c)
        except OSError:
            pass
    raise RuntimeError('libcudart 를 못 찾음 (/usr/local/cuda/lib64 확인)')


CUDA = _load_cudart()
CUDA.cudaMalloc.argtypes = [ctypes.POINTER(ctypes.c_void_p), ctypes.c_size_t]
CUDA.cudaMemcpy.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_size_t, ctypes.c_int]
CUDA.cudaFree.argtypes = [ctypes.c_void_p]


def _chk(err, what):
    if err != 0:
        raise RuntimeError('%s 실패 (cudaError %d)' % (what, err))


def cuda_malloc(nbytes):
    p = ctypes.c_void_p()
    _chk(CUDA.cudaMalloc(ctypes.byref(p), nbytes), 'cudaMalloc')
    return p


def h2d(dptr, arr):
    _chk(CUDA.cudaMemcpy(dptr, arr.ctypes.data_as(ctypes.c_void_p), arr.nbytes, H2D), 'cudaMemcpy H2D')


def d2h(arr, dptr):
    _chk(CUDA.cudaMemcpy(arr.ctypes.data_as(ctypes.c_void_p), dptr, arr.nbytes, D2H), 'cudaMemcpy D2H')


LOGGER = trt.Logger(trt.Logger.WARNING)


class TrtEngine:
    """엔진 파일을 읽어 두고 infer(blob) → 출력 배열. 스레드 하나에서만 쓸 것."""

    def __init__(self, path):
        with open(path, 'rb') as f, trt.Runtime(LOGGER) as rt:
            self.engine = rt.deserialize_cuda_engine(f.read())
        if self.engine is None:
            raise RuntimeError('엔진 읽기 실패: %s (다른 기기/TensorRT 버전에서 만든 엔진은 못 씀)' % path)
        self.ctx = self.engine.create_execution_context()
        self.io = []   # (이름, 입력?, shape, dtype, 장치포인터, 호스트배열)
        n = self.engine.num_bindings
        for i in range(n):
            name = self.engine.get_binding_name(i)
            shape = tuple(self.engine.get_binding_shape(i))
            dtype = trt.nptype(self.engine.get_binding_dtype(i))
            host = np.empty(shape, dtype=dtype)
            self.io.append((name, self.engine.binding_is_input(i), shape, dtype, cuda_malloc(host.nbytes), host))
        self.bindings = [int(x[4].value) for x in self.io]
        self.inp = [x for x in self.io if x[1]][0]
        self.out = [x for x in self.io if not x[1]][0]

    @property
    def input_shape(self):
        return self.inp[2]

    def infer(self, blob):
        blob = np.ascontiguousarray(blob, dtype=self.inp[3])
        if blob.shape != self.inp[2]:
            raise ValueError('입력 모양 %s ≠ 엔진 %s' % (blob.shape, self.inp[2]))
        h2d(self.inp[4], blob)
        if not self.ctx.execute_v2(self.bindings):
            raise RuntimeError('execute_v2 실패')
        d2h(self.out[5], self.out[4])
        return self.out[5].copy()

    def close(self):
        for x in self.io:
            CUDA.cudaFree(x[4])
        self.io = []
