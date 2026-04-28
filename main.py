# 这是一个示例 Python 脚本。
# 按 Shift+F10 执行或将其替换为您的代码。
# 按 双击 Shift 在所有地方搜索类、文件、工具窗口、操作和设置。

from src.gi_engine_class import *
from src.other_class import *
import yaml
import time
import sys
import os
import subprocess as sp
import json

def run_plot_result():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    sp.run([sys.executable, os.path.join(script_dir, "plotResult.py")], cwd=script_dir, check=True)

def imu_noise_init(options, yamldata):
    # 读取IMU误差初始值(零偏和比例因子)
    options.initstate.imuerror.gyrbias = np.array(yamldata["initgyrbias"]) * math.pi / 180 / 3600.0
    options.initstate.imuerror.accbias = np.array(yamldata["initaccbias"]) * 1e-5
    options.initstate.imuerror.gyrscale = np.array(yamldata["initgyrscale"]) * 1e-6
    options.initstate.imuerror.accscale = np.array(yamldata["initaccscale"]) * 1e-6

    # 读取初始位置、速度、姿态(欧拉角)的标准差
    options.initstate_std.pos = np.array(yamldata["initposstd"])
    options.initstate_std.vel = np.array(yamldata["initvelstd"])
    options.initstate_std.euler = np.array(yamldata["initattstd"]) * math.pi / 180

    # 读取IMU噪声参数
    options.imunoise.gyr_arw = np.array(yamldata["imunoise"]["arw"])
    options.imunoise.acc_vrw = np.array(yamldata["imunoise"]["vrw"])
    options.imunoise.gyrbias_std = np.array(yamldata["imunoise"]["gbstd"])
    options.imunoise.accbias_std = np.array(yamldata["imunoise"]["abstd"])
    options.imunoise.gyrscale_std = np.array(yamldata["imunoise"]["gsstd"])
    options.imunoise.accscale_std = np.array(yamldata["imunoise"]["asstd"])
    options.imunoise.corr_time = yamldata["imunoise"]["corrtime"]

    # 读取IMU误差初始标准差, 如果配置文件中没有设置，则采用IMU噪声参数中的零偏和比例因子的标准差
    options.initstate_std.imuerror.gyrbias = options.imunoise.gyrbias_std * math.pi / 180.0 / 3600.0
    options.initstate_std.imuerror.accbias = options.imunoise.accbias_std * 1e-5
    options.initstate_std.imuerror.gyrscale = options.imunoise.gyrscale_std * 1e-6
    options.initstate_std.imuerror.accscale = options.imunoise.accscale_std * 1e-6

    # IMU噪声参数转换为标准单位
    options.imunoise.gyr_arw = (math.pi / 180.0 / 60.0) * options.imunoise.gyr_arw
    options.imunoise.acc_vrw = options.imunoise.acc_vrw / 60.0
    options.imunoise.gyrbias_std = (math.pi / 180.0 / 3600.0) * options.imunoise.gyrbias_std
    options.imunoise.accbias_std = 1e-5 * options.imunoise.accbias_std
    options.imunoise.gyrscale_std = 1e-6 * options.imunoise.gyrscale_std
    options.imunoise.accscale_std = 1e-6 * options.imunoise.accscale_std
    options.imunoise.corr_time = 3600.0 * options.imunoise.corr_time

    # 输出文件路径
    options.navfile_path = os.path.join(yamldata["output_path"], "KF_GINS_Navresult.txt") # 组合导航结果输出路径
    options.imuerrfile_path = os.path.join(yamldata["output_path"], "KF_GINS_IMU_ERR.txt") # IMU误差 陀螺 和 加速度计 的零偏和标度因数
    options.stdfile_path = os.path.join(yamldata["output_path"], "KF_GINS_STD.txt") # 状态标准差 时间-位置-速度-姿态-零偏-标度因数

    # 创建并清空txt文件
    with open(options.navfile_path, 'w') as f:
        pass
    with open(options.imuerrfile_path, 'w') as f:
        pass
    with open(options.stdfile_path, 'w') as f:
        pass

# 初始化函数
def data_init(yamldata):
    # 文件中保存的数据，根据配置文件中设置的频率，经过降频处理了
    GNSS_RTK_Path = os.path.join(yamldata["input_path"], "GNSS.txt")
    IMU_Path = os.path.join(yamldata["input_path"], "IMU.txt")
    Truth_Path = os.path.join(yamldata["input_path"], "GT.txt")

    Gnss_data = None
    Imu_data = None
    Truth_data = None
    try:
        Gnss_data = np.loadtxt(GNSS_RTK_Path) # 每行7列，timestamp(s) xyz(m) vxyz(m/s) # imu的频率根据yaml文件中的设置进行初始化
        Imu_data = np.loadtxt(IMU_Path) # 每行7列，timestamp(s) gyroXYZ(rad/s) acceXYZ(m/s2) # gnss的频率根据文件中的输出决定，频率可以不是定值
        Truth_data = np.loadtxt(Truth_Path) # 每行14列 timestamp(s) xyz(m) vxyz(m/s) euler(xyz) 四元数(qwxyz device to world) # 参考轨迹的频率与IMU频率相同
    except Exception as e:
        print(f"Read File Error:{e}")
        quit()

    # 导航初始信息

    # IMU数据每行选取7列，数据频率是200Hz
    imudatarate = yamldata["imudatarate"]
    # 处理时间段
    startIndex = 200
    starttime = Truth_data[startIndex, 0]
    endtime = Truth_data[-1, 0]

    imufile = ImuFileLoader(Imu_data, imudatarate)
    gnssfile = GnssFileLoader(Gnss_data)

    options = GINSOptions()
    # 东北天 右前上 读取初始位置(x,y,z)、速度(vx,vy,vz)、姿态(四元数 qwxyz device to world)
    options.initstate.pos = Truth_data[startIndex, 1:4]
    options.initstate.vel = Truth_data[startIndex, 4:7]
    options.initstate.qwxyz = Truth_data[startIndex, 10:14]

    # 初始化噪声参数，以及输出文件路径
    imu_noise_init(options, yamldata)

    return options, imufile, gnssfile, starttime, endtime

# 输出导航结果
def writeNavResult(time, navstate, navfile_path, imuerrfile_path):
    result = []
    result.append(0)
    result.append(time)
    result.append(navstate.pos[0])
    result.append(navstate.pos[1])
    result.append(navstate.pos[2])
    result.append(navstate.vel[0])
    result.append(navstate.vel[1])
    result.append(navstate.vel[2])
    result.append(navstate.euler[0] * 180.0 / math.pi)
    result.append(navstate.euler[1] * 180.0 / math.pi)
    result.append(navstate.euler[2] * 180.0 / math.pi)
    result.append(navstate.qwxyz[0])
    result.append(navstate.qwxyz[1])
    result.append(navstate.qwxyz[2])
    result.append(navstate.qwxyz[3])

    try:
        with open(navfile_path, 'a', encoding='utf-8') as file:
            for item in result:
                # 写入每个元素，并添加换行符
                file.write(str(item) + " ")
            file.write('\n')
    except FileNotFoundError:
        print(f"文件 {navfile_path} 不存在。")

    imuerr = navstate.imuerror
    result = []
    result.append(time)

    result.append(imuerr.gyrbias[0])
    result.append(imuerr.gyrbias[1])
    result.append(imuerr.gyrbias[2])
    result.append(imuerr.accbias[0])
    result.append(imuerr.accbias[1])
    result.append(imuerr.accbias[2])
    result.append(imuerr.gyrscale[0])
    result.append(imuerr.gyrscale[1])
    result.append(imuerr.gyrscale[2])
    result.append(imuerr.accscale[0])
    result.append(imuerr.accscale[1])
    result.append(imuerr.accscale[2])

    try:
        with open(imuerrfile_path, 'a', encoding='utf-8') as file:
            for item in result:
                # 写入每个元素，并添加换行符
                file.write(str(item) + " ")
            file.write('\n')
    except FileNotFoundError:
        print(f"文件 {imuerrfile_path} 不存在。")

# 输出导航误差
def writeSTD(time, cov, stdfile_path):
    result = []

    result.append(time)
    # 保存位置、速度、姿态标准差
    for i in range(0, 6):
        # if cov[i, i] < 0:
        #     raise ValueError
        result.append(math.sqrt(cov[i, i]))
    for i in range(6, 9):
        # if cov[i, i] < 0:
        #     raise ValueError
        result.append(math.sqrt(cov[i, i]) * 180.0 / math.pi)

    # 保存IMU误差标准差
    # 陀螺零偏标准差
    for i in range(9, 12):
        result.append(math.sqrt(cov[i, i]))
    # 加表零偏标准差
    for i in range(12, 15):
        result.append(math.sqrt(cov[i, i]))
    for i in range(15, 21):
        result.append(math.sqrt(cov[i, i]))

    try:
        with open(stdfile_path, 'a', encoding='utf-8') as file:
            for item in result:
                # 写入每个元素，并添加换行符
                file.write(str(item) + " ")
            file.write('\n')
    except FileNotFoundError:
        print(f"文件 {stdfile_path} 不存在。")


# 主函数入口
if __name__ == '__main__':
    # 加载配置文件
    fp = "./dataset/kf-gins.yaml"
    with open(fp, 'r', encoding='utf-8') as file:
        yamldata = yaml.load(file, Loader=yaml.FullLoader)

    # 初始化参数
    options, imufile, gnssfile, starttime, endtime = data_init(yamldata)

    # 使用options初始化解算引擎
    giengine = gi_engine(options)

    if endtime < 0:
        endtime = imufile.endtime()

    # 数据对齐
    imu_cur = IMU()
    while imu_cur.time < starttime:
        imu_cur = imufile.next()
    gnss = GNSS()
    while gnss.time <= starttime:
        gnss = gnssfile.next()

    giengine.addImuData(imu_cur, True)
    giengine.addGnssData(gnss)

    # 用于显示进度
    percent = 0
    lastpercent = 0
    interval = endtime - starttime

    process_start_time = time.time()
    while True:
        # 当前IMU状态时间新于GNSS时间时，读取并添加新的GNSS数据到GIEngine
        if gnss.time < imu_cur.time:
            gnss = gnssfile.next()
            if gnss != -1:
                giengine.addGnssData(gnss)
        # 读取并添加新的IMU数据到GIEngine
        imu_cur = imufile.next()
        if imu_cur.time > endtime:
            break

        # 文件越界则退出程序
        if imu_cur == -1 or gnss == -1:
            process_end_time = time.time()
            CostTime = process_end_time - process_start_time
            print("KF-GINS Process Finish!")
            print(f"From {round(starttime, 2)} s to {round(endtime, 2)} s, total {round(interval, 2)} s !")
            print(f"Cost {round(CostTime, 2)} s in total.")
            run_plot_result()
            sys.exit()

        giengine.addImuData(imu_cur, False)

        # 处理新的IMU数据
        giengine.newImuProcess()

        # 获取当前时间，IMU状态和协方差
        timestamp = giengine.timestamp()
        navstate  = giengine.getNavState()
        cov       = giengine.getCovariance()

        # 保存处理结果
        writeNavResult(timestamp, navstate, options.navfile_path, options.imuerrfile_path)
        writeSTD(timestamp, cov, options.stdfile_path)

        # 显示运行进度
        percent = (imu_cur.time - starttime) / interval * 100
        if percent - lastpercent >= 1:
            print(f" - Processing: {round(percent, 2)}%")
            lastpercent = percent

    process_end_time = time.time()
    CostTime = process_end_time - process_start_time
    print("KF-GINS Process Finish!")
    print(f"From {round(starttime, 2)} s to {round(endtime, 2)} s, total {round(interval, 2)} s !")
    print(f"System cost {round(CostTime, 2)} s in total.")

    run_plot_result()


