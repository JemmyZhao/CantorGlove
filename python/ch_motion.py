from euclid3 import *
from transmitter import *

GRAVITY = Vector3(0, 0, 1)


class PhysicalBody:
    def __init__(self):
        self.pos = Vector3()
        self.quat = Quaternion()


class StaticBody(PhysicalBody):
    def __init__(self):
        PhysicalBody.__init__(self)


class DynamicBody(PhysicalBody):
    def __init__(self):
        PhysicalBody.__init__(self)
        self.acc = Vector3()
        self.gyro = Vector3()


class Drum(StaticBody):
    def __init__(self, n=GRAVITY, yaw_rangle=[-180,0]):
        StaticBody.__init__(self)
        self.normal = n
        self.yaw_range = yaw_rangle


class DrumGroup:
    def __init__(self):
        self.drum_1 = Drum(yaw_rangle=[-180, 0])
        self.drum_2 = Drum(yaw_rangle=[0, 180])
        self.drum_3 = Drum(yaw_rangle=[-180, 0])
        self.drum_4 = Drum(yaw_rangle=[0, 180])


class Hand:
    def __init__(self):
        self.acc_earth = Vector3(0, 0, 0)
        self.acc_sensor = Vector3(0, 0, 0)
        self.gyro_earth = Vector3(0, 0, 0)
        self.gyro_sensor = Vector3()
        self.quat = Quaternion(1, 0, 0, 0)
        self.pos = Vector3(0, 0, 0)

    def set_AHRS(self, ahrs):
        if isinstance(ahrs, AHRSMsg):
            self.quat = Quaternion(ahrs.q0, ahrs.q1, ahrs.q2, ahrs.q3)
            self.gyro_sensor = Vector3(ahrs.gx, ahrs.gy, ahrs.gz)*180/np.pi
            a = ahrs.get_acc_earth()
            self.acc_sensor = ahrs.get_acc()
            self.acc_earth = a - GRAVITY
            self.gyro_earth = self.quat * self.gyro_sensor


class Hands:
    def __init__(self):
        self.right_hand = Hand()
        self.left_hand = Hand()

#
# def get_hit(hands, drum_group):
#     if isinstance(hands, Hands) and isinstance(drum_group, DrumGroup):
