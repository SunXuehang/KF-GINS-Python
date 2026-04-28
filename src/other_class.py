import copy
import sys
import numpy as np
import math
from scipy.spatial.transform import Rotation as R
import quaternion

class ImuError:
    def __init__(self):
        self.gyrbias = np.zeros((1, 3))
        self.accbias = np.zeros((1, 3))
        self.gyrscale = np.zeros((1, 3))
        self.accscale = np.zeros((1, 3))

class NavState:
    def __init__(self):
        self.pos = np.zeros((1, 3)) # 导航坐标系的相对位移
        self.posBlh = np.zeros((1, 3)) # 纬经高
        self.vel = np.zeros((1, 3))
        self.euler = np.zeros((1, 3))
        self.qwxyz = np.zeros((1, 4)) # qwxyz device to world
        self.imuerror = ImuError()

class ImuNoise:
    def __init__(self):
        self.gyr_arw = np.zeros((1, 3))
        self.acc_vrw = np.zeros((1, 3))
        self.gyrbias_std = np.zeros((1, 3))
        self.accbias_std = np.zeros((1, 3))
        self.gyrscale_std = np.zeros((1, 3))
        self.accscale_std = np.zeros((1, 3))
        self.corr_time = 0

class GINSOptions:
    def __init__(self):
        self.initstate = NavState()
        self.initstate_std = NavState()
        self.imunoise = ImuNoise()
        self.imuerror = ImuError()
        self.antlever = np.zeros((1, 3))

        self.navfile_path = " "
        self.imuerrfile_path = " "
        self.stdfile_path = " "


class IMU:
    def __init__(self):
        self.time = 0
        self.dt = 0
        self.omega = np.zeros((3, ))  # 角速度
        self.f = np.zeros((3, ))  # 比力
        self.odovec = 0

class GNSS:
    def __init__(self):
        self.time = 0
        self.xyz = np.zeros((3, ))  # xyz位置
        self.vxyz = np.zeros((3,))  # xyz速度
        self.std = np.zeros((3, ))  # 位置标准差
        self.isvalid = False

class ImuFileLoader:
    def __init__(self, ImuData, rate):
        self.ImuData = np.array(ImuData)
        self.FileIndexMax = self.ImuData.__len__()
        self.FileIndex = 0
        self.dt_ = 1.0 / rate
        self.imu_ = IMU()
        self.imu_pre_ = IMU()

    def next(self):

        if self.FileIndex == 0:
            self.imu_.time = self.ImuData[self.FileIndex, 0]
            self.imu_.dtheta = self.ImuData[self.FileIndex, 1:4]
            self.imu_.dvel = self.ImuData[self.FileIndex, 4:7]
            self.imu_.dt = self.dt_
            self.FileIndex += 1
            return self.imu_
        else:
            if self.FileIndex >= self.FileIndexMax:
                print(f'ImuData has no {self.FileIndex} line! So KF-GINS Process Finish!')
                return -1
            self.imu_pre_ = copy.deepcopy(self.imu_)
            self.imu_.time = self.ImuData[self.FileIndex, 0]
            self.imu_.omega = self.ImuData[self.FileIndex, 1:4]
            self.imu_.f = self.ImuData[self.FileIndex, 4:7]

            self.FileIndex += 1

            dt = self.imu_.time - self.imu_pre_.time
            if dt < 0.1:
                self.imu_.dt = dt
            else:
                self.imu_.dt = self.dt_

            return self.imu_

    def endtime(self):
        time = self.ImuData[-1:, 0]
        float_time = time[0]
        return float_time

class GnssFileLoader:
    def __init__(self, GnssData):
        self.gnss_ = GNSS()
        self.FileIndex = 0
        self.FileIndexMax = GnssData.__len__()
        self.GnssData = np.array(GnssData)

    def next(self):
        self.FileIndex += 1
        if self.FileIndex >= self.FileIndexMax:
            print(f'GNSSData has no {self.FileIndex} line! So KF-GINS Process Finish!')
            return -1
        self.gnss_.time = self.GnssData[self.FileIndex, 0]
        self.gnss_.xyz = self.GnssData[self.FileIndex, 1:4]
        self.gnss_.vxyz = self.GnssData[self.FileIndex, 4:7]
        self.gnss_.std = np.array([0.1, 0.1, 0.1, 0.001, 0.001, 0.001])

        return self.gnss_

class Attitude:
    def __init__(self):
        # 四元数 qwxyz device to world
        self.qbn = np.quaternion(1, 0, 0, 0)
        # 旋转矩阵
        self.cbn = np.zeros((3, 3))
        # 欧拉角 返回的欧拉角顺序是绕载体坐标系的xyz轴旋转的顺序，也意味着不同的载体坐标系，返回的偏航抚养横滚顺序不同
        self.euler = np.zeros((3,))


class PVA:
    def __init__(self):
        self.pos = np.zeros((1, 3))
        self.vel = np.zeros((1, 3))
        self.att = Attitude()

class Rotation:
    def __init__(self):
        self.Cbn = np.zeros((3, 3))

    @staticmethod
    def rotvec2quaternion(rotvec):
        rotation = R.from_rotvec(rotvec.tolist())
        vec = rotation.as_quat()

        return np.quaternion(vec[3], vec[0], vec[1], vec[2])

    @staticmethod
    def quaternion2vector(quaternion):
        rotation = R.from_quat([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        return rotation.as_rotvec()

    @staticmethod
    def quaternion2matrix(quaternion):
        rotation = R.from_quat([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        return rotation.as_matrix()

    @staticmethod
    def matrix2euler(dcm):
        euler = np.zeros((3, ))


        euler[1] = math.atan(-dcm[2, 0] / math.sqrt(dcm[2, 1] * dcm[2, 1] + dcm[2, 2] * dcm[2, 2]))

        if dcm[2, 0] <= -0.999:
            euler[0] = 0
            euler[2] = math.atan2((dcm[1, 2] - dcm[0, 1]), (dcm[0, 2] + dcm[1, 1]))
        elif dcm[2, 0] >= 0.999:
            euler[0] = 0
            euler[2] = math.pi + math.atan2((dcm[1, 2] + dcm[0, 1]), (dcm[0, 2] - dcm[1, 1]))
        else:
            euler[0] = math.atan2(dcm[2, 1], dcm[2, 2])
            euler[2] = math.atan2(dcm[1, 0], dcm[0, 0])

        if euler[2] < 0:
            euler[2] = math.pi * 2 + euler[2]

        return euler

    @staticmethod
    def euler2matrix(euler):
        matrix_x = np.array([[1, 0, 0], [0, math.cos(euler[0]), math.sin(euler[0])], [0, -math.sin(euler[0]), math.cos(euler[0])]])
        matrix_y = np.array([[math.cos(euler[1]), 0, -math.sin(euler[1])], [0, 1, 0], [math.sin(euler[1]), 0, math.cos(euler[1])]])
        matrix_z = np.array([[math.cos(euler[2]), math.sin(euler[2]), 0], [-math.sin(euler[2]), math.cos(euler[2]), 0], [0, 0, 1]])

        Cnb = np.dot(np.dot(matrix_y, matrix_x), matrix_z)
        Cbn = Cnb.T

        # 返回导航系b到载体系n的坐标转换矩阵
        return Cbn

    @staticmethod
    def euler2quaternion(euler):
        rot_z = euler[2]
        rot_x = euler[0]
        rot_y = euler[1]

        q0 = np.cos(rot_z / 2) * np.cos(rot_y / 2) * np.cos(rot_x / 2) + np.sin(rot_z / 2) * np.sin(rot_y / 2) * np.sin(rot_x / 2)
        q1 = np.cos(rot_z / 2) * np.cos(rot_y / 2) * np.sin(rot_x / 2) - np.sin(rot_z / 2) * np.sin(rot_y / 2) * np.cos(rot_x / 2)
        q2 = np.sin(rot_z / 2) * np.cos(rot_y / 2) * np.sin(rot_x / 2) + np.cos(rot_z / 2) * np.sin(rot_y / 2) * np.cos(rot_x / 2)
        q3 = np.sin(rot_z / 2) * np.cos(rot_y / 2) * np.cos(rot_x / 2) - np.cos(rot_z / 2) * np.sin(rot_y / 2) * np.sin(rot_x / 2)

        # yaw = euler[2]
        # roll = euler[1]
        # pitch = euler[0]
        #
        # q0 = np.cos(yaw / 2) * np.cos(pitch / 2) * np.cos(roll / 2) - np.sin(yaw / 2) * np.sin(pitch / 2) * np.sin(roll / 2)
        # q1 = np.cos(yaw / 2) * np.sin(pitch / 2) * np.cos(roll / 2) - np.sin(yaw / 2) * np.cos(pitch / 2) * np.sin(roll / 2)
        # q2 = np.cos(yaw / 2) * np.cos(pitch / 2) * np.sin(roll / 2) + np.sin(yaw / 2) * np.sin(pitch / 2) * np.cos(roll / 2)
        # q3 = np.cos(yaw / 2) * np.sin(pitch / 2) * np.sin(roll / 2) + np.sin(yaw / 2) * np.cos(pitch / 2) * np.cos(roll / 2)

        qbn = np.array([q0, q1, q2, q3])

        # rotation = R.from_euler('zyx', [yaw, pitch, roll], degrees=False)
        # q = rotation.as_quat()

        return np.quaternion(qbn[0], qbn[1], qbn[2], qbn[3])

    @staticmethod
    def skewSymmetric(vector):
        mat = [[0, -vector[2], vector[1]],
               [vector[2], 0, -vector[0]],
               [-vector[1], vector[0], 0]]

        aaa = np.array(mat)

        return np.array(mat)
class Earth:
    def __init__(self):
        self.name = "Earth"
        self.WGS84_WIE = 0
        self.WGS84_F   = 0.0033528106647474805
        self.WGS84_RA  = 6378137.0000000000
        self.WGS84_RB  = 6356752.3142451793
        self.WGS84_GM0 = 398600441800000.00
        self.WGS84_E1  = 0.0066943799901413156
        self.WGS84_E2  = 0.0067394967422764341

    def meridianPrimeVerticalRadius(self, lat):
        tmp = math.sin(lat)
        tmp *= tmp
        tmp = 1 - self.WGS84_E1 * tmp
        sqrttmp = math.sqrt(tmp)

        res = np.array([self.WGS84_RA * (1 - self.WGS84_E1) / (sqrttmp * tmp), self.WGS84_RA / sqrttmp])

        return res

    def DRi(self, blh):
        dri = np.zeros((3, 3))

        rmn = self.meridianPrimeVerticalRadius(blh[0])

        dri[0, 0] = 1.0 / (rmn[0] + blh[2])
        dri[1, 1] = 1.0 / ((rmn[1] + blh[2]) * math.cos(blh[0]))
        dri[2, 2] = -1

        return dri

    def DR(self, blh):
        dr = np.zeros((3, 3))

        rmn = self.meridianPrimeVerticalRadius(blh[0])

        dr[0, 0] = rmn[0] + blh[2]
        dr[1, 1] = (rmn[1] + blh[2]) * math.cos(blh[0])
        dr[2, 2] = -1

        return dr

    def qne(self, blh):
        quat = np.array([1.0, 0.0, 0.0, 0.0])

        coslon = math.cos(blh[1] * 0.5)
        sinlon = math.sin(blh[1] * 0.5)
        coslat = math.cos(-math.pi * 0.25 - blh[0] * 0.5)
        sinlat = math.sin(-math.pi * 0.25 - blh[0] * 0.5)

        quat[3] = coslat * coslon
        quat[0] = -sinlat * sinlon
        quat[1] = sinlat * coslon
        quat[2] = coslat * sinlon

        return np.quaternion(quat[3], quat[0], quat[1], quat[2])

    def blh(self, qne, height):
        blh_res = np.zeros((3, ))

        blh_res[0] = -2 * math.atan(qne.y / qne.w) - math.pi * 0.5
        blh_res[1] = 2 * math.atan2(qne.z, qne.w)
        blh_res[2] = height

        return blh_res

    def gravity(self, blh):
        sin2 = math.sin(blh[0])
        sin2 *= sin2

        return 9.7803267715 * (1 + 0.0052790414 * sin2 + 0.0000232718 * sin2 * sin2) + \
            blh[2] * (0.0000000043977311 * sin2 - 0.0000030876910891) + 0.0000000000007211 * blh[2] * blh[2]








