"""官方 V2.3.1beta3 C API 的窄适配器，供 H1/H2 比较 Python GIL 阻塞。

只绑定随 DLL 提供的 jakaAPI.h / jktypes.h 声明。ctypes.CDLL 在调用时释放
GIL；不改变 servo_j 算法、8ms 周期、安全检查或现场门槛。全局GUI尚未采用。
没有上电、使能、复位、TCP/负载修改接口，不能用于绕开现场检查。
"""
import ctypes as C
import hashlib
import os
import time
from pathlib import Path

from ..jaka import checked

I=C.c_int
P=C.POINTER


class Vector6(C.Structure):
    _fields_=[('values',C.c_double*6)]


class Payload(C.Structure):
    _fields_=[('mass',C.c_double),('centroid',C.c_double*3),('payload_id',I)]


class MotionStatus(C.Structure):
    _fields_=[('values',I*11)]


class SimpleStatus(C.Structure):
    _fields_=[('errcode',I),('errmsg',C.c_char*200),('powered',I),('enabled',I)]


class EdgIO(C.Structure):
    _fields_=[('cab_din',I*16),('cab_dout',I*16),('cab_ain',C.c_double*2),
              ('cab_aout',C.c_double*2),('tool_din',I*2),('tool_dout',I*2),
              ('tool_ain',C.c_double*2)]


class EdgState(C.Structure):
    _fields_=[('q',Vector6),('v',Vector6),('torque',Vector6),('tcp',Vector6),
              ('sensor',Vector6),('io',EdgIO)]


def abi_layout():
    return {cls.__name__:{'size':C.sizeof(cls),'alignment':C.alignment(cls),
                        'offsets':{name:getattr(cls,name).offset for name,*_ in cls._fields_}}
            for cls in (Vector6,Payload,MotionStatus,SimpleStatus,EdgIO,EdgState)}


class NativeRC:
    binding='official_c_api_ctypes_gil_released_candidate'

    def __init__(self,directory,host,queue_api=False,edg_servo=False):
        if os.name!='nt' or C.sizeof(C.c_void_p)!=8:
            raise RuntimeError('native candidate requires Windows x64')
        directory=Path(directory).resolve();self.host=host;self.handle=I();self.logged=False
        if queue_api and edg_servo:raise ValueError('EDG0 cannot use normal queue API')
        self.queue_api=queue_api;self.edg_servo=edg_servo;self.trace=None
        self.commissioning_edg_mode=0 if edg_servo else 1
        self.command_acceptance_scope='sdk_local_return_only' if edg_servo else 'sdk_normal_return'
        self.dll_path=directory/'jakaAPI.dll'
        self.dll_sha256=hashlib.sha256(self.dll_path.read_bytes()).hexdigest()
        self.dll_directory=os.add_dll_directory(str(directory))
        self.dll=C.CDLL(str(self.dll_path))
        signatures={
            'create_handler':[C.c_char_p,P(I)],'destory_handler':[P(I)],
            'get_motion_status':[P(I),P(MotionStatus)],
            'get_robot_status_simple':[P(I),P(SimpleStatus)],
            'get_actual_joint_position':[P(I),P(Vector6)],
            'get_actual_tcp_position':[P(I),P(Vector6)],
            'get_tool_id':[P(I),P(I)],'get_user_frame_id':[P(I),P(I)],'get_collision_level':[P(I),P(I)],
            'is_in_pos':[P(I),P(I)],'is_in_servomove':[P(I),P(I)],
            'get_tool_data':[P(I),I,P(Vector6)],'get_payload':[P(I),P(Payload)],
            'get_sdk_version':[P(I),P(C.c_char)],
            'kine_forward':[P(I),P(Vector6),P(Vector6)],
            'set_network_exception_handle':[P(I),C.c_float,I],
            'edg_init_extend':[P(I),I,C.c_char_p,I,I],
            'edg_get_stat':[P(I),P(EdgState),C.c_ubyte],
            'edg_stat_details':[P(I),P(C.c_ulong)],
            'servo_move_enable':[P(I),I],
            'servo_j_extend':[P(I),P(Vector6),I,C.c_uint],
            'servo_j_extend_queue':[P(I),P(Vector6),I,C.c_uint,P(I),C.c_void_p,I],
            'edg_servo_j_extend':[P(I),P(Vector6),I,C.c_uint,C.c_ubyte],
            'motion_abort':[P(I)]}
        self.functions={}
        for name,args in signatures.items():
            f=getattr(self.dll,name);f.argtypes=args;f.restype=I;self.functions[name]=f

    def _call(self,name,*args):
        if not self.logged:raise RuntimeError('native SDK is not owned/logged in')
        start=time.perf_counter();code=None
        try:
            code=int(self.functions[name](C.byref(self.handle),*args));return code
        finally:
            if getattr(self,'trace',None) and name in ('get_motion_status','get_robot_status_simple',
                                                     'servo_j_extend','servo_j_extend_queue','edg_servo_j_extend'):
                self.trace(event='native_call',name=name,started_s=start,returned_s=time.perf_counter(),code=code)

    def login(self):
        if self.logged:raise RuntimeError('native SDK already owned')
        code=int(self.functions['create_handler'](self.host.encode('ascii'),C.byref(self.handle)))
        if code==0:self.logged=True
        return (code,)

    def logout(self):
        code=self._call('destory_handler');self.logged=False;return (code,)

    def _integer(self,name):
        value=I();code=self._call(name,C.byref(value));return (code,value.value)

    def is_in_servomove(self):return self._integer('is_in_servomove')
    def is_in_pos(self):return self._integer('is_in_pos')
    def get_tool_id(self):return self._integer('get_tool_id')
    def get_user_frame_id(self):return self._integer('get_user_frame_id')
    def get_collision_level(self):return self._integer('get_collision_level')

    def _vector(self,name):
        value=Vector6();code=self._call(name,C.byref(value));return (code,list(value.values))

    def get_actual_joint_position(self):return self._vector('get_actual_joint_position')
    def get_actual_tcp_position(self):return self._vector('get_actual_tcp_position')

    def get_motion_status(self):
        value=MotionStatus();code=self._call('get_motion_status',C.byref(value))
        return (code,list(value.values))

    def get_robot_status_simple(self):
        value=SimpleStatus();code=self._call('get_robot_status_simple',C.byref(value))
        return (code,[value.errcode,value.errmsg.decode('utf-8',errors='replace'),value.powered,value.enabled])

    def read_safety(self):
        return {'motion':self.get_motion_status(),'simple':self.get_robot_status_simple()}

    def get_tool_data(self,tool_id):
        value=Vector6();code=self._call('get_tool_data',tool_id,C.byref(value))
        return (code,tool_id,list(value.values))

    def get_payload(self):
        value=Payload();code=self._call('get_payload',C.byref(value))
        return (code,[value.mass,list(value.centroid)])

    def get_sdk_version(self):
        value=C.create_string_buffer(1024);code=self._call('get_sdk_version',value)
        return (code,value.value.decode('ascii',errors='replace'))

    def kine_forward(self,q):
        value=Vector6();joint=Vector6((C.c_double*6)(*q))
        code=self._call('kine_forward',C.byref(joint),C.byref(value))
        return (code,list(value.values))

    def set_network_exception_handle(self,ms,action):
        return (self._call('set_network_exception_handle',float(ms),action),)

    def edg_init_extend(self,on,ip,port,mode):
        return (self._call('edg_init_extend',int(on),ip.encode('ascii'),port,mode),)

    def edg_get_stat(self,robot_id):
        value=EdgState();code=self._call('edg_get_stat',C.byref(value),robot_id)
        io=[list(getattr(value.io,name)) for name,*_ in EdgIO._fields_]
        return (code,[list(value.q.values),list(value.v.values),list(value.torque.values),
                      list(value.tcp.values),list(value.sensor.values),io])

    def edg_stat_details(self):
        value=(C.c_ulong*3)();code=self._call('edg_stat_details',value)
        return (code,list(value))

    def servo_move_enable(self,on):return (self._call('servo_move_enable',int(on)),)
    def servo_j(self,q,mode,step):
        if len(q)!=6 or mode!=0 or step!=1:
            raise ValueError('native commissioning adapter only accepts absolute six-joint step_num=1')
        joint=Vector6((C.c_double*6)(*q))
        if getattr(self,'edg_servo',False):
            return (self._call('edg_servo_j_extend',C.byref(joint),mode,step,0),)
        if getattr(self,'queue_api',False):
            depth=I(-1)
            code=self._call('servo_j_extend_queue',C.byref(joint),mode,step,C.byref(depth),None,0)
            return (code,depth.value)
        return (self._call('servo_j_extend',C.byref(joint),mode,step),)

    def motion_abort(self):return (self._call('motion_abort'),)
