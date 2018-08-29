import serial
import struct
import numpy as np
import euclid3


def byte2int(buffers, bias):
    x = struct.unpack('i', buffers[bias:bias+4])
    return x[0]


def byte2float(buffers, bias):
    x = struct.unpack('f', buffers[bias:bias+4])
    return x[0]

# Mavlink Msg
class Msg:
    def __init__(self, com_name):
        self.ser = serial.Serial(com_name, 115200, timeout=1)
        self.stx = 0
        self.data_len = 0
        self.msg_id = 0
        self.data = []

    def read(self):
        while ord(self.ser.read(1)) != 0xfe:
            start = ord(self.ser.read(1))
            if start == 0xfe:
                break
        self.data_len = int(ord(self.ser.read(1)))
        self.msg_id = int(ord(self.ser.read(1)))
        self.data = self.ser.read(self.data_len)

class AHRSMsg(Msg):
    def __init__(self, com_name):
        Msg.__init__(self, com_name)
        self.timestamps = 0
        self.ax = 0
        self.ay = 0
        self.az = 0
        self.gx = 0
        self.gy = 0
        self.gz = 0
        self.q0 = 0
        self.q1 = 0
        self.q2 = 0
        self.q3 = 0

    def read(self):
        Msg.read(self)
        # print(self.data)
        self.timestamps = byte2float(self.data, 0)
        self.ax = byte2float(self.data, 4)
        self.ay = byte2float(self.data, 8)
        self.az = byte2float(self.data, 12)
        self.gx = byte2float(self.data, 16)
        self.gy = byte2float(self.data, 20)
        self.gz = byte2float(self.data, 24)
        self.q0 = byte2float(self.data, 28)
        self.q1 = byte2float(self.data, 32)
        self.q2 = byte2float(self.data, 36)
        self.q3 = byte2float(self.data, 40)

    def get_acc(self):
        return euclid3.Vector3(self.ax, self.ay, self.az)

    def get_gyro(self):
        return euclid3.Vector3(self.gx, self.gy, self.gz)

    def get_quat(self):
        return euclid3.Quaternion(self.q0, self.q1, self.q2, self.q3)

    # Euler in deg
    def get_euler(self):
        q = euclid3.Quaternion(self.q0, self.q1, self.q2, self.q3)
        euler = q.get_euler()
        return euclid3.Vector3(euler[2] * 57.29577951, euler[0] * 57.29577951, euler[1] * 57.29577951)

    def get_acc_earth(self):
        q = euclid3.Quaternion(self.q0, self.q1, self.q2, self.q3)
        a = euclid3.Vector3(self.ax, self.ay, self.az)
        e = q*a
        return euclid3.Vector3(e.x, e.y, e.z)







