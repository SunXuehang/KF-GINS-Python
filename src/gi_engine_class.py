from src.INSMech import *
import copy

class gi_engine:
    def __init__(self, options):
        self.options_ = copy.deepcopy(options)
        self.timestamp_ = 0

        self.TIME_ALIGN_ERR = 0.001

        self.imupre_ = IMU()
        self.imucur_ = IMU()
        self.gnssdata_ = GNSS()

        self.pvacur_ = PVA()
        self.pvapre_ = PVA()
        self.imuerror_ = ImuError()

        self.RANK = 21
        self.NOISERANK = 18

        self.Cov_ = np.zeros((self.RANK, self.RANK))
        self.Qc_ = np.zeros((self.NOISERANK, self.NOISERANK))
        self.dx_= np.zeros((self.RANK, ))

        # StateID
        self.P_ID = 0
        self.V_ID = 3
        self.PHI_ID = 6
        self.BG_ID = 9
        self.BA_ID = 12
        self.SG_ID = 15
        self.SA_ID = 18

        # NoiseID
        self.VRW_ID = 0
        self.ARW_ID = 3
        self.BGSTD_ID = 6
        self.BASTD_ID = 9
        self.SGSTD_ID = 12
        self.SASTD_ID = 15

        self.rotation = Rotation()

        self.init_gi_engine()

        self.insmech = INSMech()

        self.earth = Earth()

    def init_gi_engine(self):
        imunoise = copy.deepcopy(self.options_.imunoise)
        self.Qc_[self.ARW_ID:(self.ARW_ID + 3), self.ARW_ID:(self.ARW_ID + 3)] = np.diag(imunoise.gyr_arw) @ np.diag(imunoise.gyr_arw)
        self.Qc_[self.VRW_ID:(self.VRW_ID + 3), self.VRW_ID:(self.VRW_ID + 3)] = np.diag(imunoise.acc_vrw) @ np.diag(imunoise.acc_vrw)
        self.Qc_[self.BGSTD_ID:(self.BGSTD_ID + 3), self.BGSTD_ID:(self.BGSTD_ID + 3)] = (2 / imunoise.corr_time) * np.diag(imunoise.gyrbias_std) @ np.diag(imunoise.gyrbias_std)
        self.Qc_[self.BASTD_ID:(self.BASTD_ID + 3), self.BASTD_ID:(self.BASTD_ID + 3)] = (2 / imunoise.corr_time) * np.diag(imunoise.accbias_std) @ np.diag(imunoise.accbias_std)
        self.Qc_[self.SGSTD_ID:(self.SGSTD_ID + 3), self.SGSTD_ID:(self.SGSTD_ID + 3)] = (2 / imunoise.corr_time) * np.diag(imunoise.gyrscale_std) @ np.diag(imunoise.gyrscale_std)
        self.Qc_[self.SASTD_ID:(self.SASTD_ID + 3), self.SASTD_ID:(self.SASTD_ID + 3)] = (2 / imunoise.corr_time) * np.diag(imunoise.accscale_std)  @ np.diag(imunoise.accscale_std)

        # 设置系统状态(位置、速度、姿态和IMU误差)初值和初始协方差
        initstate = copy.deepcopy(self.options_.initstate)
        self.pvacur_.pos = 1.0 * copy.deepcopy(initstate.pos)
        self.pvacur_.vel = copy.deepcopy(initstate.vel)
        self.pvacur_.att.qbn = np.quaternion(initstate.qwxyz[0], initstate.qwxyz[1], initstate.qwxyz[2],
                                             initstate.qwxyz[3])
        self.pvacur_.att.cbn = Rotation.quaternion2matrix(self.pvacur_.att.qbn)
        self.pvacur_.att.euler = Rotation.matrix2euler(self.pvacur_.att.cbn)

        self.imuerror_ = copy.deepcopy(initstate.imuerror)

        self.pvapre_ = copy.deepcopy(self.pvacur_)

        initstate_std = copy.deepcopy(self.options_.initstate_std)
        imuerror_std = copy.deepcopy(self.options_.initstate_std.imuerror)
        # 初始P矩阵设大一点，便于快速收敛
        self.Cov_[self.P_ID:(self.P_ID + 3), self.P_ID:(self.P_ID + 3)] = 4 * np.diag(initstate_std.pos) @ np.diag(initstate_std.pos)
        self.Cov_[self.V_ID:(self.V_ID + 3), self.V_ID:(self.V_ID + 3)] = 4 * np.diag(initstate_std.vel) @ np.diag(initstate_std.vel)
        self.Cov_[self.PHI_ID:(self.PHI_ID + 3), self.PHI_ID:(self.PHI_ID + 3)] = 4 * np.diag(initstate_std.euler) @ np.diag(initstate_std.euler)
        self.Cov_[self.BG_ID:(self.BG_ID + 3), self.BG_ID:(self.BG_ID + 3)] = 4 * np.diag(imuerror_std.gyrbias) @ np.diag(imuerror_std.gyrbias)
        self.Cov_[self.BA_ID:(self.BA_ID + 3), self.BA_ID:(self.BA_ID + 3)] = 4 * np.diag(imuerror_std.accbias) @ np.diag(imuerror_std.accbias)
        self.Cov_[self.SG_ID:(self.SG_ID + 3), self.SG_ID:(self.SG_ID + 3)] = 4 * np.diag(imuerror_std.gyrscale) @ np.diag(imuerror_std.gyrscale)
        self.Cov_[self.SA_ID:(self.SA_ID + 3), self.SA_ID:(self.SA_ID + 3)] = 4 * np.diag(imuerror_std.accscale) @ np.diag(imuerror_std.accscale)


    def imuCompensate(self, imu):
        imu.omega = imu.omega - self.imuerror_.gyrbias
        imu.f = imu.f - self.imuerror_.accbias

        gyrscale = np.ones((3, )) + self.imuerror_.gyrscale
        accscale = np.ones((3, )) + self.imuerror_.accscale

        # gyrscale = np.ones((3, ))
        # accscale = np.ones((3, ))

        imu.omega = np.multiply(imu.omega, np.reciprocal(gyrscale))
        imu.f = np.multiply(imu.f, np.reciprocal(accscale))

    def addImuData(self, imu, compensate):
        self.imupre_ = copy.deepcopy(self.imucur_)
        self.imucur_ = copy.deepcopy(imu)

        if compensate:
            self.imuCompensate(self.imucur_)

    def addGnssData(self, gnss):
        self.gnssdata_ = copy.deepcopy(gnss)
        self.gnssdata_.isvalid = True

    def isToUpdate(self, imutime1, imutime2, updatetime):
        if abs(imutime1 - updatetime) < self.TIME_ALIGN_ERR:
            # 更新时间靠近imutime1
            # GNSS数据靠近上一历元，先对上一历元进行GNSS更新
            return 1
        elif abs(imutime2 - updatetime) <= self.TIME_ALIGN_ERR:
            # 更新时间靠近imutime2
            # GNSS数据靠近当前历元，先对当前IMU进行状态传播
            return 2
        elif imutime1 < updatetime and updatetime < imutime2:
            # 更新时间在imutime1和imutime2之间, 但不靠近任何一个
            return 3
        else:
            # 更新时间不在imutimt1和imutime2之间，且不靠近任何一个
            # 只传播导航状态
            return 0

    def insPropagation(self, imupre, imucur):
        self.imuCompensate(self.imucur_)
        self.insmech.insMech(self.pvapre_, self.pvacur_, imupre, imucur)

        # 初始化状态传播矩阵
        F = np.zeros_like(self.Cov_)
        G = np.zeros((self.RANK, self.NOISERANK))

        # 使用上一历元状态计算状态转移矩阵
        accel = imucur.f
        omega = imucur.omega

        # 位置误差
        F[self.P_ID:(self.P_ID + 3), self.V_ID:(self.V_ID + 3)] = np.eye(3)

        # 速度误差
        F[self.V_ID:(self.V_ID + 3), self.PHI_ID:(self.PHI_ID + 3)] = Rotation.skewSymmetric(self.pvapre_.att.cbn @ accel)
        F[self.V_ID:(self.V_ID + 3), self.BA_ID:(self.BA_ID + 3)] = self.pvapre_.att.cbn
        F[self.V_ID:(self.V_ID + 3), self.SA_ID:(self.SA_ID + 3)] = self.pvapre_.att.cbn @ np.diag(accel)

        # 姿态误差
        F[self.PHI_ID:(self.PHI_ID+3), self.BG_ID:(self.BG_ID+3)] = -self.pvapre_.att.cbn
        F[self.PHI_ID:(self.PHI_ID+3), self.SG_ID:(self.SG_ID+3)] = -self.pvapre_.att.cbn @ (np.diag(omega))

        # IMU零偏误差和比例因子误差，建模成一阶高斯 - 马尔科夫过程
        F[self.BG_ID:(self.BG_ID+3), self.BG_ID:(self.BG_ID+3)] = -1 / self.options_.imunoise.corr_time * np.eye(3)
        F[self.BA_ID:(self.BA_ID+3), self.BA_ID:(self.BA_ID+3)] = -1 / self.options_.imunoise.corr_time * np.eye(3)
        F[self.SG_ID:(self.SG_ID+3), self.SG_ID:(self.SG_ID+3)] = -1 / self.options_.imunoise.corr_time * np.eye(3)
        F[self.SA_ID:(self.SA_ID+3), self.SA_ID:(self.SA_ID+3)] = -1 / self.options_.imunoise.corr_time * np.eye(3)

        # 系统噪声驱动矩阵
        G[self.V_ID:(self.V_ID+3), self.VRW_ID:(self.VRW_ID+3)] = self.pvapre_.att.cbn
        G[self.PHI_ID:(self.PHI_ID+3), self.ARW_ID:(self.ARW_ID+3)] = self.pvapre_.att.cbn
        G[self.BG_ID:(self.BG_ID+3), self.BGSTD_ID:(self.BGSTD_ID+3)] = np.eye(3)
        G[self.BA_ID:(self.BA_ID+3), self.BASTD_ID:(self.BASTD_ID+3)] = np.eye(3)
        G[self.SG_ID:(self.SG_ID+3), self.SGSTD_ID:(self.SGSTD_ID+3)] = np.eye(3)
        G[self.SA_ID:(self.SA_ID+3), self.SASTD_ID:(self.SASTD_ID+3)] = np.eye(3)

        # 状态转移矩阵
        Phi = np.eye(self.Cov_.__len__())
        Phi = Phi + F * imucur.dt

        Qd = G @ self.Qc_ @ G.T * imucur.dt
        Qd = (Phi @ Qd @ Phi.T + Qd) / 2

        # EKF预测传播系统协方差和系统误差状态
        self.EKFPredict(Phi, Qd)

    def EKFPredict(self, Phi, Qd):
        # 传播系统协方差和误差状态
        self.Cov_ = Phi @ self.Cov_ @ Phi.T + Qd
        self.dx_ = Phi @ self.dx_

    def EKFUpdate(self, dz, H, R):
        temp = H @ self.Cov_ @ H.T + R
        K = self.Cov_ @ H.T @ np.linalg.inv(temp)

        I = np.eye(self.Cov_.__len__())

        # 更新系统误差状态和协方差
        I = I - K @ H

        # 如果每次更新后都进行状态反馈，则更新前dx_一直为0，下式可以简化为：dx_ = K * dz
        self.dx_ = self.dx_ + K @ (dz - H @ self.dx_)
        self.Cov_ = I @ self.Cov_ @ I.T + K @ R @ K.T

    def gnssUpdate(self, gnssdata):
        # GNSS位置测量新息
        dz1 = self.pvacur_.pos - gnssdata.xyz
        dz2 = self.pvacur_.vel - gnssdata.vxyz
        dz = np.concatenate((dz1, dz2), axis=0)

        # 构造GNSS位置观测矩阵
        H_gnsspos = np.zeros((6, self.Cov_.__len__()))
        H_gnsspos[0:3, self.P_ID:(self.P_ID + 3)] = np.eye(3)
        H_gnsspos[3:6, self.V_ID:(self.V_ID + 3)] = np.eye(3)

        # 位置观测噪声阵
        R_gnsspos = np.diag(gnssdata.std) @ np.diag(gnssdata.std)

        # EKF更新协方差和误差状态
        self.EKFUpdate(dz, H_gnsspos, R_gnsspos)

        # GNSS更新之后设置为不可用
        gnssdata.isvalid = False

    def stateFeedback(self):
        # 位置误差反馈
        delta_r = self.dx_[self.P_ID:(self.P_ID + 3)]
        self.pvacur_.pos -= delta_r

        # 速度误差反馈
        vectemp = self.dx_[self.V_ID:(self.V_ID + 3)]
        self.pvacur_.vel -= vectemp

        # 姿态误差反馈
        vectemp = self.dx_[self.PHI_ID:(self.PHI_ID + 3)]
        qpn = Rotation.rotvec2quaternion(vectemp)
        self.pvacur_.att.qbn = qpn * self.pvacur_.att.qbn
        self.pvacur_.att.cbn = Rotation.quaternion2matrix(self.pvacur_.att.qbn)
        self.pvacur_.att.euler = Rotation.matrix2euler(self.pvacur_.att.cbn)

        # IMU零偏误差反馈
        vectemp = self.dx_[self.BG_ID:(self.BG_ID + 3)]
        self.imuerror_.gyrbias += vectemp
        vectemp = self.dx_[self.BA_ID:(self.BA_ID + 3)]
        self.imuerror_.accbias += vectemp

        # IMU比例因子误差反馈
        vectemp = self.dx_[self.SG_ID:(self.SG_ID + 3)]
        self.imuerror_.gyrscale += vectemp
        vectemp = self.dx_[self.SA_ID:(self.SA_ID + 3)]
        self.imuerror_.accscale += vectemp

        # 误差状态反馈到系统状态后, 将误差状态清零
        self.dx_ = np.zeros_like(self.dx_)

    # IMU插值
    def imuInterpolate(self, imu1, imu2, timestamp, midimu):
        if imu1.time > timestamp or imu2.time < timestamp:
            return

        lamda = (timestamp - imu1.time) / (imu2.time - imu1.time)

        midimu.time   = timestamp
        midimu.omega = imu1.omega + (imu2.omega - imu1.omega) * lamda
        midimu.f   = imu1.f + (imu2.f - imu1.f) * lamda
        midimu.dt     = timestamp - imu1.time

    def checkCov(self):
        try:
            for i in range(0, self.RANK):
                if self.Cov_[i, i] < 0:
                    raise ValueError("Covariance is negative at" + str(self.timestamp_))
        except ValueError as e:
            print(str(e))

    def timestamp(self):
        return self.timestamp_

    def getNavState(self):
        state = NavState()

        state.pos = self.pvacur_.pos
        state.vel = self.pvacur_.vel
        state.euler = self.pvacur_.att.euler
        state.qwxyz = np.asarray([self.pvacur_.att.qbn.w, self.pvacur_.att.qbn.x, self.pvacur_.att.qbn.y, self.pvacur_.att.qbn.z])
        state.imuerror = self.imuerror_

        return state

    def getCovariance(self):
        return self.Cov_

    def newImuProcess(self):
        self.timestamp_ = copy.deepcopy(self.imucur_.time)

        if self.gnssdata_.isvalid:
            self.updatetime = copy.deepcopy(self.gnssdata_.time)
        else:
            self.updatetime = -1

        res = self.isToUpdate(self.imupre_.time, self.imucur_.time, self.updatetime)

        if res == 0:
            self.insPropagation(self.imupre_, self.imucur_)
        elif res == 1:
            # GNSS数据靠近上一历元，先对上一历元进行GNSS更新
            self.gnssUpdate(self.gnssdata_)
            self.stateFeedback()

            self.pvapre_ = copy.deepcopy(self.pvacur_)
            self.insPropagation(self.imupre_, self.imucur_)
        elif res == 2:
            # GNSS数据靠近当前历元，先对当前IMU进行状态传播
            self.insPropagation(self.imupre_, self.imucur_)
            self.gnssUpdate(self.gnssdata_)
            self.stateFeedback()
        else:
            # GNSS数据在两个IMU数据之间(不靠近任何一个), 将当前IMU内插到整秒时刻
            midimu = IMU()
            self.imuInterpolate(self.imupre_, self.imucur_, self.updatetime, midimu)

            # 对前一半IMU进行状态传播
            self.insPropagation(self.imupre_, midimu)

            # 整秒时刻进行GNSS更新，并反馈系统状态
            self.gnssUpdate(self.gnssdata_)
            self.stateFeedback()

            # 对后一半IMU进行状态传播
            self.pvapre_ = self.pvacur_
            self.insPropagation(midimu, self.imucur_)

        # 检查协方差矩阵对角线元素
        self.checkCov()

        # 更新上一时刻的状态和IMU数据
        self.pvapre_ = copy.deepcopy(self.pvacur_)
        self.imupre_ = copy.deepcopy(self.imucur_)