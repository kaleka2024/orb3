"""
 * This file is part of ORB-SLAM3
 *
 * Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 * Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 *
 * ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
 * the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with ORB-SLAM3.
 * If not, see <http://www.gnu.org/licenses/>.
"""
import sys          # 系统命令行参数模块
import numpy as np  # 数值计算数组库
import matplotlib.pyplot as plt  # 绘图可视化库


class dataset:
    # imu_sync = np.zeros((1,7))
    # acc = np.zeros((1,7))
    # gyro = np.zeros((1,7))

    def __init__(self, dirName):
        """
        @brief 数据集类构造函数，读取原始数据集文件
        @param dirName 数据集根目录路径
        """
        self.name = dirName                     # 保存数据集根目录字符串
        self.acc = np.zeros((1, 4))             # 加速度计数组：[timestamp, ax, ay, az]
        self.gyro = np.zeros((1, 4))            # 陀螺仪数组：[timestamp, gx, gy, gz]
        self.timesCam = np.zeros((1, 1))        # 相机图像时间戳数组

        # 读取相机时间戳文件 cam0/times.txt
        timesName = self.name + "/cam0/times.txt"
        timeFile = open(timesName, "r")
        i = 0
        next = 0
        for line in timeFile:
            currentline = line.split(",")      # 按逗号分割一行文本
            if i % 2 == 0:                     # 隔行读取，只取偶数行
                self.timesCam[next] = currentline
                next = next + 1
                # 数组向后填充一行0，动态扩容
                self.timesCam = np.pad(
                    self.timesCam, ((0, 1), (0, 0)), mode="constant", constant_values=0
                )
            i = i + 1
            print(i, "/", next)              # 打印读取进度
        timeFile.close()

        # 读取加速度计IMU文件 IMU/acc.txt
        accName = self.name + "/IMU/acc.txt"
        accFile = open(accName, "r")
        i = 0
        for line in accFile:
            currentline = line.split(",")
            for j in range(0, 4):
                self.acc[i][j] = currentline[j]  # timestamp,ax,ay,az
            # 数组尾部补0实现动态扩容
            self.acc = np.pad(
                self.acc, ((0, 1), (0, 0)), mode="constant", constant_values=0
            )
            i = i + 1
        accFile.close()

        # 读取陀螺仪IMU文件 IMU/gyro.txt
        gyroName = self.name + "/IMU/gyro.txt"
        gyroFile = open(gyroName, "r")
        i = 0
        for line in gyroFile:
            currentline = line.split(",")
            for j in range(0, 4):
                self.gyro[i][j] = currentline[j]  # timestamp,gx,gy,gz
            # 数组尾部补0实现动态扩容
            self.gyro = np.pad(
                self.gyro, ((0, 1), (0, 0)), mode="constant", constant_values=0
            )
            i = i + 1
        gyroFile.close()

        # 删除最后一行多余的填充0行
        self.timesCam = np.delete(self.timesCam, self.timesCam.shape[0] - 1, axis=0)
        self.acc = np.delete(self.acc, self.acc.shape[0] - 1, axis=0)
        self.gyro = np.delete(self.gyro, self.gyro.shape[0] - 1, axis=0)

        print("Finished")

    def interpolate(self):
        """
        @brief IMU数据同步插值：陀螺仪原始采样点作为基准时间，对加速度做线性插值
        @details imuSync每一行：[timestamp,gx,gy,gz,ax,ay,az]
        输入acc、gyro时间戳不一致；输出gyro时间戳对齐，加速度插值到gyro时刻
        """
        self.imuSync = np.zeros((self.gyro.shape[0], 7))
        print("shape = ", self.imuSync.shape)

        totAcc = self.acc.shape[0]      # 加速度计总样本数
        totGyro = self.gyro.shape[0]    # 陀螺仪总样本数

        idxAcc = 0                      # acc数组遍历索引
        idxGyro = 0                     # gyro数组遍历索引
        print(self.acc[idxAcc][0])
        print(self.gyro[idxGyro][0])
        # 找到第一个acc时间戳大于gyro时间戳的位置
        while self.acc[idxAcc][0] > self.gyro[idxGyro][0]:
            idxGyro = idxGyro + 1

        idxSync = 0                     # imuSync输出数组索引
        while idxAcc + 1 < totAcc and idxGyro < totGyro:
            # 插值计算变量：当前acc相邻两个采样点时间差、加速度差值
            deltaTimeAcc = self.acc[idxAcc + 1, 0] - self.acc[idxAcc, 0]
            deltaAcc = self.acc[idxAcc + 1, 1:4] - self.acc[idxAcc, 1:4]
            # 在当前acc两个时间点之间，遍历所有gyro采样点
            while (
                    idxGyro < totGyro and self.acc[idxAcc + 1, 0] >= self.gyro[idxGyro, 0]
            ):
                self.imuSync[idxSync, 0] = self.gyro[idxGyro, 0]
                # 线性插值计算该gyro时刻对应的加速度
                self.imuSync[idxSync, 4:7] = (
                        self.acc[idxAcc, 1:4]
                        + (self.gyro[idxGyro, 0] - self.acc[idxAcc, 0])
                        * deltaAcc
                        / deltaTimeAcc
                )

                # 陀螺仪数据直接复制，不插值
                self.imuSync[idxSync, 1:4] = self.gyro[idxGyro, 1:4]

                idxGyro = idxGyro + 1
                idxSync = idxSync + 1

            idxAcc = idxAcc + 1
        # 截断输出数组，删掉未使用的尾部0行
        self.imuSync = np.delete(self.imuSync, range(idxSync, totGyro), axis=0)

    def plotGyro(self):
        """@brief 绘制同步之后陀螺仪角速度曲线"""
        for i in range(1, 4):
            plt.plot(self.imuSync[:, 0], self.imuSync[:, i], label=str("acc ") + str(i))
        plt.xlabel("time (s)")
        plt.ylabel("ang. vel. (rad/s)")
        plt.title("Gyroscope")
        plt.legend()
        plt.show()

    def plotAcc(self):
        """@brief 绘制同步之后加速度计曲线"""
        for i in range(4, 7):
            plt.plot(self.imuSync[:, 0], self.imuSync[:, i], label=str("acc ") + str(i))
        plt.xlabel("time (s)")
        plt.ylabel("acc (m/s^2)")
        plt.title("Accelerometer")
        plt.legend()
        plt.show()

    def saveSynchronized(self):
        """
        @brief 将插值同步完成的IMU数据保存为 imu0.csv，适配ORB‑SLAM3惯性数据集格式
        @note 时间戳转换：秒 → 纳秒 ×1e9，输出csv表头与Euroc格式保持一致
        """
        imuName = self.name + "/imu0.csv"
        imuFile = open(imuName, "w")
        imuFile.write(
            "#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]\n"
        )
        for row in self.imuSync:
            i = 0
            for num in row:
                if i == 0:
                    imuFile.write(str((int)(1e9 * num)))  # 秒转纳秒整数
                    i = 1
                else:
                    imuFile.write("," + str(num))
            imuFile.write("\n")
        imuFile.close()

    def saveCorrectTimes(self):
        """@brief 导出相机时间戳到 cam0/corrTimes.txt"""
        timesName = self.name + "/cam0/corrTimes.txt"
        timesFile = open(timesName, "w")
        print("self.timesCam shape ", self.timesCam.shape)
        for row in self.timesCam:
            i = 0
            for num in row:
                timesFile.write(str((int)(num)))
                timesFile.write("\n")
        timesFile.close()


if __name__ == "__main__":
    # 命令行参数校验：只允许 2个或3个参数
    if len(sys.argv) != 2 and len(sys.argv) != 3:
        print("Number of arguments != 2 and 3")
        sys.exit()

    dirName = sys.argv[1]
    print("Processing :", dirName)

    myDataset = dataset(dirName)         # 实例化数据集对象，加载原始文件
    myDataset.interpolate()              # 执行IMU加速度线性插值同步
    myDataset.plotAcc()                  # 绘制加速度计波形图
    myDataset.saveSynchronized()         # 保存同步后的imu0.csv文件

    # 如果传入第三个参数，则额外输出校正后的相机时间戳文件
    if len(sys.argv) == 3:
        myDataset.saveCorrectTimes()
