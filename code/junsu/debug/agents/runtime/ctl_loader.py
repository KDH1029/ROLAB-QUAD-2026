"""go2_ctl.py 를 ROS 없이 불러오는 로더 — rclpy·메시지 모듈을 가짜로 채운 뒤 실제 파일을 import 한다(노드·DDS 안 만듦)."""
import importlib.util, os, sys, types

CTL = os.path.expanduser('~/ws/junsu_v2/tools/go2_ctl.py')


def _stub(name, **attrs):
    m = types.ModuleType(name); m.__dict__.update(attrs); sys.modules[name] = m; return m


class _Any:
    def __init__(self, *a, **k): pass
    def __getattr__(self, k): return _Any()
    def __call__(self, *a, **k): return _Any()


def load(with_unitree=True):
    class Node:            # Mon 의 부모 — 하네스에선 Mon 을 만들지 않는다
        def __init__(self, *a, **k): raise RuntimeError('하네스: Mon(Node) 생성 금지')
    _stub('rclpy', init=lambda *a, **k: None, spin=lambda *a, **k: None, try_shutdown=lambda *a, **k: None)
    _stub('rclpy.node', Node=Node)
    _stub('rclpy.qos', QoSProfile=_Any, ReliabilityPolicy=_Any(), HistoryPolicy=_Any())
    _stub('std_msgs'); _stub('std_msgs.msg', String=type('String', (), {}))
    _stub('sensor_msgs'); _stub('sensor_msgs.msg', PointCloud2=type('PointCloud2', (), {}), Imu=type('Imu', (), {}))
    _stub('nav_msgs'); _stub('nav_msgs.msg', Odometry=type('Odometry', (), {}))
    _stub('rcl_interfaces'); _stub('rcl_interfaces.srv', SetParameters=_Any)
    _stub('rcl_interfaces.msg', Parameter=_Any, ParameterValue=_Any, ParameterType=_Any())
    if with_unitree:
        _stub('unitree_api'); _stub('unitree_api.msg', Request=_Any)
        _stub('unitree_go'); _stub('unitree_go.msg', LowState=_Any, SportModeState=_Any)
    as_path = os.environ.get('HARNESS_CTL_AS', CTL)      # 패치 사본을 진짜 자리(tools/)에 있는 것처럼(J 가 맞게)
    mod = types.ModuleType('go2_ctl_under_test'); mod.__file__ = as_path
    exec(compile(open(CTL).read(), as_path, 'exec'), mod.__dict__)
    return mod
