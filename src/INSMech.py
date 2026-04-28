from .other_class import *
import numpy as np

class INSMech:
    def __init__(self):
        self.name = "IMUMesh"
        self.earth = Earth()

    def insMech(self, pvapre, pvacur, imupre, imucur):
        self.attUpdate(pvapre, pvacur, imupre, imucur)
        self.velUpdate(pvapre, pvacur, imupre, imucur)
        self.posUpdate(pvapre, pvacur, imupre, imucur)

    def posUpdate(self, pvapre, pvacur, imupre, imucur):
        midvel = (pvacur.vel + pvapre.vel) / 2
        pvacur.pos = pvapre.pos + midvel * imucur.dt

    def velUpdate(self, pvapre, pvacur, imupre, imucur):
        # 这个重力加速度是，程序简化之前，初始时刻计算的重力加速度大小（由原始程序debug获取）
        gravity = 9.8

        # b系比力积分项
        d_vfb = (imucur.f + imupre.f) / 2 * imucur.dt
        # d_vfb = imucur.f * imucur.dt

        # 比力积分项投影到n系
        d_vfn = pvapre.att.cbn @ d_vfb

        # 重力投影到n系
        gl = np.array([0, 0, -gravity])
        d_vgn = gl * imucur.dt

        # 速度更新完成
        pvacur.vel = pvapre.vel + d_vfn + d_vgn

    def attUpdate(self, pvapre, pvacur, imupre, imucur):
        temp1 = np.array([
            [0, -imupre.omega[0], -imupre.omega[1], -imupre.omega[2]],
            [imupre.omega[0], 0, imupre.omega[2], -imupre.omega[1]], 
            [imupre.omega[1], -imupre.omega[2], 0, imupre.omega[0]],
            [imupre.omega[2], imupre.omega[1], -imupre.omega[0], 0]
        ])
        temp2 = np.array([
            [0, -imucur.omega[0], -imucur.omega[1], -imucur.omega[2]],
            [imucur.omega[0], 0, imucur.omega[2], -imucur.omega[1]], 
            [imucur.omega[1], -imucur.omega[2], 0, imucur.omega[0]],
            [imucur.omega[2], imucur.omega[1], -imucur.omega[0], 0]
        ])

        q = np.zeros((4, ))
        q[0] = pvacur.att.qbn.w
        q[1] = pvacur.att.qbn.x
        q[2] = pvacur.att.qbn.y
        q[3] = pvacur.att.qbn.z

        K1 = 0.5 * temp1 @ q
        K2 = 0.5 * temp2 @ q

        q = q + (K1 + K2) * imucur.dt / 2

        pvacur.att.qbn.w = q[0]
        pvacur.att.qbn.x = q[1]
        pvacur.att.qbn.y = q[2]
        pvacur.att.qbn.z = q[3]
        pvacur.att.cbn = Rotation.quaternion2matrix(pvacur.att.qbn)
        pvacur.att.euler = Rotation.matrix2euler(pvacur.att.cbn)

        pass


