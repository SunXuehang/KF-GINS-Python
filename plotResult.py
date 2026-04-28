import numpy as np
import matplotlib.pyplot as plt
import yaml
import time
import sys
import os
from matplotlib.collections import LineCollection

if __name__ == "__main__":
    fig_size = (12, 10)
    fig_dpi = 150

    # 加载配置文件
    fp = "./dataset/kf-gins.yaml"
    with open(fp, 'r', encoding='utf-8') as file:
        yamldata = yaml.load(file, Loader=yaml.FullLoader)
    gt_path = os.path.join(yamldata["input_path"], "GT.txt")
    output_path = yamldata["output_path"]

    # 读取数据文件
    Nav_ans_txt = np.loadtxt(os.path.join(output_path, "KF_GINS_Navresult.txt"))
    Std_txt = np.loadtxt(os.path.join(output_path, "KF_GINS_STD.txt"))
    error_bias_scale = np.loadtxt(os.path.join(output_path, "KF_GINS_IMU_ERR.txt"))
    ground_truth = np.loadtxt(gt_path)

    # 删除 Nav_ans_txt 的第一列
    Nav_ans_txt = Nav_ans_txt[:, 1:]

    # 计算数据长度
    length = len(Nav_ans_txt)

    # 找到真实值与导航结果对应的时间起点
    start_index = np.where(np.abs(ground_truth[:, 0] - Nav_ans_txt[0, 0]) < 0.005)[0][0]

    # 剔除真实值中的重复时间点
    truth_txt = [ground_truth[start_index, :]]
    for i in range(start_index + 1, len(ground_truth)):
        if ground_truth[i, 0] - ground_truth[i - 1, 0] > 0:
            truth_txt.append(ground_truth[i, :])
    truth_txt = np.array(truth_txt)

    # 计算误差
    def is_positive(x, y):
        return np.abs(x - y) <= 0.005

    error = np.zeros((length, 14))
    index = 0
    for i in range(length):
        index = np.where(np.abs(truth_txt[:, 0] - Nav_ans_txt[i, 0]) < 0.005)[0][0]
        error[i, 0] = Nav_ans_txt[i, 0]
        error[i, 1:] = Nav_ans_txt[i, 1:] - truth_txt[index, 1:]

        # 处理误差角 0 到 360 度范围
        for j in range(7, 10):
            if 360 - error[i, j] < 15:
                error[i, j] -= 360
            elif error[i, j] + 360 < 15:
                error[i, j] += 360

    error = error[:-1, :]

    # 误差图
    fig, axes = plt.subplots(3, 3, figsize=fig_size, dpi=fig_dpi)

    for i in range(3):
        axes[0, i].plot(error[:, 0], error[:, i + 1])
        axes[0, i].plot(Std_txt[:, 0], 3 * Std_txt[:, i + 1], 'r')
        axes[0, i].plot(Std_txt[:, 0], -3 * Std_txt[:, i + 1], 'r')
        axes[0, i].set_xlabel("time(s)")
        axes[0, i].legend(["position error", "3σ"])

    for i in range(3):
        axes[1, i].plot(error[:, 0], error[:, i + 4])
        axes[1, i].plot(Std_txt[:, 0], 3 * Std_txt[:, i + 4], 'r')
        axes[1, i].plot(Std_txt[:, 0], -3 * Std_txt[:, i + 4], 'r')
        axes[1, i].set_xlabel("time(s)")
        axes[1, i].legend(["velocity error", "3σ"])

    for i in range(3):
        axes[2, i].plot(error[:, 0], error[:, i + 7])
        axes[2, i].plot(Std_txt[:, 0], 3 * Std_txt[:, i + 7], 'r')
        axes[2, i].plot(Std_txt[:, 0], -3 * Std_txt[:, i + 7], 'r')
        axes[2, i].set_xlabel("time(s)")
        axes[2, i].legend(["attitude error", "3σ"])

    fig.savefig(os.path.join(output_path, "KF_GINS_Result.png"), dpi=fig_dpi)

    # 注意：error 已经 error = error[:-1, :]
    Nav_plot = Nav_ans_txt[:len(error), :]

    # 计算水平定位误差，也可以改成三维误差
    pos_error = np.linalg.norm(error[:, 1:4], axis=1)
    # pos_error_3d = np.linalg.norm(error[:, 1:4], axis=1) # 三维位置误差

    x = Nav_plot[:, 1]
    y = Nav_plot[:, 2]

    # 构造轨迹线段
    points = np.array([x, y]).T.reshape(-1, 1, 2)
    segments = np.concatenate([points[:-1], points[1:]], axis=1)

    fig, ax = plt.subplots(figsize=fig_size, dpi=fig_dpi)

    # 根据误差大小给每一段轨迹上色
    lc = LineCollection(
        segments,
        cmap="jet",
        norm=plt.Normalize(pos_error.min(), pos_error.max())
    )
    lc.set_array(pos_error[:-1])
    lc.set_linewidth(5)

    line = ax.add_collection(lc)

    # 绘制真值轨迹
    ax.plot(
        truth_txt[:, 1],
        truth_txt[:, 2],
        "k--",
        linewidth=1.5,
        label="Ground Truth Pos"
    )

    ax.scatter(truth_txt[0, 1], truth_txt[0, 2], color='green', marker='o', label='Start Point')
    ax.scatter(truth_txt[-1, 1], truth_txt[-1, 2], color='red', marker='x', label='End Point')

    ax.grid()
    ax.axis("equal")
    ax.legend()
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title("Trajectory Colored by Position Error")

    # 添加颜色条
    cbar = fig.colorbar(line, ax=ax)
    cbar.set_label("Position Error (m)")

    fig.savefig(os.path.join(output_path, "KF_GINS_Trajectory_Error_Color.png"), dpi=fig_dpi)
    plt.show()
