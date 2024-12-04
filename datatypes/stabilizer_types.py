from ctypes import c_uint32, Structure, Union, c_float
from dataclasses import dataclass



## Abstract base class for c_like vec3 structures
class Vec3_s(Structure):
    _fields_ = [
        ("timestamp", c_uint32),
        ("x", c_float),
        ("y", c_float),
        ("z", c_float)
    ]

    def to_dataclass(self) -> "Vec3DataClass":
        raise NotImplementedError("Subclasses must implement this method.")

## Abstract base class for vec3 dataclasses
@dataclass
class Vec3DataClass():
    timestamp: int = 0
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    def to_struct(self) -> Vec3_s:
        raise NotImplementedError("Subclasses must implement this method.")




class Point_t(Vec3_s):
    def to_dataclass(self) -> Vec3DataClass:
        return PointDataClass(
            timestamp = self.timestamp,
            x = self.x,
            y = self.y,
            z = self.z
        )

@dataclass
class PointDataClass(Vec3DataClass):
    def to_struct(self) -> Vec3_s:
        return Point_t(
            timestamp = self.timestamp,
            x = self.x,
            y = self.y,
            z = self.z
        )




class Velocity_t(Vec3_s):
    def to_dataclass(self) -> Vec3DataClass:
        return VelocityDataClass(
            timestamp = self.timestamp,
            x = self.x,
            y = self.y,
            z = self.z
        )

@dataclass
class VelocityDataClass(Vec3DataClass):
    def to_struct(self) -> Vec3_s:
        return Velocity_t(
            timestamp = self.timestamp,
            x = self.x,
            y = self.y,
            z = self.z
        )
    



class Acc_t(Vec3_s):
    def to_dataclass(self) -> Vec3DataClass:
        return AccDataClass(
            timestamp = self.timestamp,
            x = self.x,
            y = self.y,
            z = self.z
        )

@dataclass
class AccDataClass(Vec3DataClass):
    def to_struct(self) -> Vec3_s:
        return Acc_t(
            timestamp = self.timestamp,
            x = self.x,
            y = self.y,
            z = self.z
        )




## Attitude in euler angle form 
class Attitude_t(Structure):
    _fields_ = [
        ("timestamp", c_uint32),
        ("roll", c_float),
        ("pitch", c_float),
        ("yaw", c_float)
    ]

    ## Forward reference to AttitudeDataClass
    def to_dataclass(self) -> "AttitudeDataClass":
        return AttitudeDataClass(
            timestamp = self.timestamp,
            roll = self.roll,
            pitch = self.pitch,
            yaw = self.yaw
        )

@dataclass
class AttitudeDataClass():
    timestamp: int = 0
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0

    def to_struct(self) -> Attitude_t:
        return Attitude_t(
            timestamp = self.timestamp,
            roll = self.roll,
            pitch = self.pitch,
            yaw = self.yaw
        )




class Q(Structure):
    _fields_ = [
        ('q0', c_float),
        ('q1', c_float),
        ('q2', c_float),
        ('q3', c_float)
    ]

    def to_dataclass(self) -> "QDataClass":
        return QDataClass(
            q0 = self.q0,
            q1 = self.q1,
            q2 = self.q2,
            q3 = self.q3,
        )

@dataclass
class QDataClass():
    q0: float = 0.0
    q1: float = 0.0
    q2: float = 0.0
    q3: float = 0.0

    def to_struct(self) -> Q:
        return Q(
            q0 = self.q0,
            q1 = self.q1,
            q2 = self.q2,
            q3 = self.q3,
        )




class X(Structure):
    _fields_ = [
        ('x', c_float),
        ('y', c_float),
        ('z', c_float),
        ('w', c_float)
    ]

    def to_dataclass(self) -> "XDataClass":
        return XDataClass(
            x = self.x,
            y = self.y,
            z = self.z,
            w = self.w 
        )

@dataclass
class XDataClass():
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    w: float = 0.0

    def to_struct(self) -> X:
        return X(
            x = self.x,
            y = self.y,
            z = self.z,
            w = self.w 
        )
    



## Orientation as a quaternion 
class Quaternion_t(Union):
    _fields_ = [
        ("orient_q", Q),
        ("orient_x", X)
    ]
    


class State_t(Structure):
    _fields_ = [
        ("attitude", Attitude_t),
        ("attitudeQuaternion", Quaternion_t),
        ("position", Point_t),
        ("velocity", Velocity_t),
        ("acc", Acc_t)
    ]



class Setpoint_t(Structure):
    _fields_ = [
        ("attitude", Attitude_t),
        ("attitudeRate", Attitude_t),
        ("attitudeQuaternion", Quaternion_t),
        ("thrust", c_float),
        ("position", Point_t),
        ("velocity", Velocity_t),
        ("acc", Acc_t)
    ]
