# Modified by Raul Mur-Artal
# Automatically compute the optimal scale factor for monocular VO/SLAM.

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Juergen Sturm, TUM
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of TUM nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Requirements:
# sudo apt-get install python-argparse
"""
This script computes the absolute trajectory error from the ground truth trajectory and the estimated trajectory.
"""

import sys                      # 系统模块，用于程序异常退出
import numpy                    # 数值计算库，矩阵、SVD分解、误差计算
import argparse                 # 命令行参数解析
import associate                # 导入同目录下associate.py，做时间戳匹配


def align(model,data):
    """Align two trajectories using the method of Horn (closed-form).

    Input:
    model -- first trajectory (3xn)
    data -- second trajectory (3xn)

    Output:
    rot -- rotation matrix (3x3)
    transGT -- translation vector (3x1) 带自动尺度优化的平移
    trans_errorGT -- translational error per point (1xn) 带尺度校正的每帧平移误差
    trans -- translation vector (3x1) 固定尺度=1的平移
    trans_error -- translational error per point (1xn) 固定尺度=1每帧平移误差
    s -- 求解得到的最优尺度因子（单目SLAM用）
    """
    # 设置numpy打印选项：小数保留3位，不输出科学计数法
    numpy.set_printoptions(precision=3,suppress=True)
    # 将model轨迹减去自身质心，做零中心化
    model_zerocentered = model - model.mean(1)
    # 将data(真值)轨迹减去自身质心，做零中心化
    data_zerocentered = data - data.mean(1)

    # 初始化3×3矩阵W，用于Horn算法计算协方差矩阵
    W = numpy.zeros( (3,3) )
    # 遍历每一个匹配位姿点，累加外积得到W矩阵
    for column in range(model.shape[1]):
        W += numpy.outer(model_zerocentered[:,column],data_zerocentered[:,column])
    # 对W的转置做SVD奇异值分解，U、Vh为正交矩阵，d为奇异值
    U,d,Vh = numpy.linalg.linalg.svd(W.transpose())
    # 初始化S为单位矩阵，用于保证旋转矩阵行列式=1（右手坐标系）
    S = numpy.matrix(numpy.identity( 3 ))
    # 如果U*Vh行列式为负，说明得到镜像，把S[2,2]置-1修正
    if(numpy.linalg.det(U) * numpy.linalg.det(Vh)<0):
        S[2,2] = -1
    # 计算旋转矩阵 R = U*S*Vh
    rot = U*S*Vh

    # 将零中心化model经过旋转矩阵变换
    rotmodel = rot*model_zerocentered
    dots = 0.0       # 点积累加，用于求解尺度s
    norms = 0.0      # 模长平方累加，用于求解尺度s

    # 遍历所有匹配点，计算点积与模长平方和
    for column in range(data_zerocentered.shape[1]):
        dots += numpy.dot(data_zerocentered[:,column].transpose(),rotmodel[:,column])
        normi = numpy.linalg.norm(model_zerocentered[:,column])
        norms += normi*normi
    # 闭式求解最优尺度因子 s = dot / norms，单目VO/SLAM核心
    s = float(dots/norms)

    # 带尺度s的平移向量：真值质心 − s*R*model质心
    transGT = data.mean(1) - s*rot * model.mean(1)
    # 不使用求解尺度，强制尺度=1的平移向量
    trans = data.mean(1) - rot * model.mean(1)

    # 使用求解尺度s对齐后的model轨迹
    model_alignedGT = s*rot * model + transGT
    # 尺度固定为1对齐后的model轨迹
    model_aligned = rot * model + trans

    # 带尺度校正的对齐误差
    alignment_errorGT = model_alignedGT - data
    # 无尺度校正的对齐误差
    alignment_error = model_aligned - data

    # 计算每一点平移误差L2范数，得到一维误差数组
    trans_errorGT = numpy.sqrt(numpy.sum(numpy.multiply(alignment_errorGT,alignment_errorGT),0)).A[0]
    trans_error = numpy.sqrt(numpy.sum(numpy.multiply(alignment_error,alignment_error),0)).A[0]

    return rot,transGT,trans_errorGT,trans,trans_error, s


def plot_traj(ax,stamps,traj,style,color,label):
    """
    Plot a trajectory using matplotlib.

    Input:
    ax -- the plot axes对象
    stamps -- time stamps (1xn) 时间戳列表
    traj -- trajectory (3xn) 轨迹坐标
    style -- line style 线条样式
    color -- line color 线条颜色
    label -- plot legend 图例标签
    """
    # 对时间戳做升序排序
    stamps.sort()
    # 计算帧间隔中位数，用来判断轨迹是否间断
    interval = numpy.median([s-t for s,t in zip(stamps[1:],stamps[:-1])])
    x = []
    y = []
    last = stamps[0]
    # 遍历每一个时间戳，分段绘制，时间跳变大于2倍中位数则断开线条
    for i in range(len(stamps)):
        if stamps[i]-last < 2*interval:
            x.append(traj[i][0])
            y.append(traj[i][1])
        elif len(x)>0:
            ax.plot(x,y,style,color=color,label=label)
            label=""
            x=[]
            y=[]
        last= stamps[i]
    # 绘制最后一段轨迹
    if len(x)>0:
        ax.plot(x,y,style,color=color,label=label)


if __name__=="__main__":
    # parse command line
    # 创建命令行参数解析器，脚本功能：计算绝对轨迹误差ATE
    parser = argparse.ArgumentParser(description='''
    This script computes the absolute trajectory error from the ground truth trajectory and the estimated trajectory.
    ''')
    # 位置参数1：真值轨迹文件，格式 timestamp tx ty tz qx qy qz qw
    parser.add_argument('first_file', help='ground truth trajectory (format: timestamp tx ty tz qx qy qz qw)')
    # 位置参数2：算法估计轨迹文件
    parser.add_argument('second_file', help='estimated trajectory (format: timestamp tx ty tz qx qy qz qw)')
    # 可选参数：给第二条轨迹时间戳增加时间偏移，默认0.0
    parser.add_argument('--offset', help='time offset added to the timestamps of the second file (default: 0.0)',default=0.0)
    # 可选参数：手动设置估计轨迹缩放系数，默认1.0；单目可交给align自动求解
    parser.add_argument('--scale', help='scaling factor for the second trajectory (default: 1.0)',default=1.0)
    # 可选参数：时间戳匹配最大允许时间差，单位纳秒，默认20000000ns=0.02s
    parser.add_argument('--max_difference', help='maximally allowed time difference for matching entries (default: 10000000 ns)',default=20000000)
    # 可选参数：保存对齐后的估计轨迹到磁盘
    parser.add_argument('--save', help='save aligned second trajectory to disk (format: stamp2 x2 y2 z2)')
    # 可选参数：保存匹配好的真值与对齐后的估计位姿对
    parser.add_argument('--save_associations', help='save associated first and aligned second trajectory to disk (format: stamp1 x1 y1 z1 stamp2 x2 y2 z2)')
    # 可选参数：输出轨迹对比png图片
    parser.add_argument('--plot', help='plot the first and the aligned second trajectory to an image (format: png)')
    # 可选参数‑verbose：打印全套误差统计（RMSE、均值、中位数、标准差、最大最小误差）
    parser.add_argument('--verbose', help='print all evaluation data (otherwise, only the RMSE absolute translational error in meters after alignment will be printed)', action='store_true')
    # 可选参数‑verbose2：同时打印开启尺度校正与不开启尺度校正两套RMSE
    parser.add_argument('--verbose2', help='print scale eror and RMSE absolute translational error in meters after alignment with and without scale correction', action='store_true')
    # 解析命令行参数
    args = parser.parse_args()

    # 读取真值轨迹文件，remove_bounds=False，不去除首尾行
    first_list = associate.read_file_list(args.first_file, False)
    # 读取算法估计轨迹文件
    second_list = associate.read_file_list(args.second_file, False)

    # 调用associate模块做时间戳匹配，得到匹配对列表[(t1,t2), ...]
    matches = associate.associate(first_list, second_list,float(args.offset),float(args.max_difference))
    # 如果匹配点少于2对，无法做对齐，直接退出报错
    if len(matches)<2:
        sys.exit("Couldn't find matching timestamp pairs between groundtruth and estimated trajectory! Did you choose the correct sequence?")
    # 提取真值匹配点的xyz，构造3×N矩阵，每一列是一个3D位置
    first_xyz = numpy.matrix([[float(value) for value in first_list[a][0:3]] for a,b in matches]).transpose()
    # 提取估计轨迹匹配点xyz，乘以用户指定的scale，构造3×N矩阵
    second_xyz = numpy.matrix([[float(value)*float(args.scale) for value in second_list[b][0:3]] for a,b in matches]).transpose()
    # 获取估计轨迹字典全部items
    dictionary_items = second_list.items()
    # 将估计轨迹按时间戳排序
    sorted_second_list = sorted(dictionary_items)

    # 全部估计轨迹点（不只是匹配点），乘以用户scale，构造3×N完整轨迹矩阵
    second_xyz_full = numpy.matrix([[float(value)*float(args.scale) for value in sorted_second_list[i][1][0:3]] for i in range(len(sorted_second_list))]).transpose() # sorted_second_list.keys()]).transpose()
    # 调用Horn对齐函数，求解旋转、平移、最优尺度、两套误差
    rot,transGT,trans_errorGT,trans,trans_error, scale = align(second_xyz,first_xyz)

    # 使用自动求解scale对齐后的匹配子集估计轨迹
    second_xyz_aligned = scale * rot * second_xyz + trans
    # scale强制=1对齐后的匹配子集估计轨迹
    second_xyz_notscaled = rot * second_xyz + trans
    # scale强制=1对齐后的完整全部估计轨迹
    second_xyz_notscaled_full = rot * second_xyz_full + trans
    # 获取真值全部时间戳并排序
    first_stamps = first_list.keys()
    first_stamps.sort()
    # 真值完整3×N位置矩阵
    first_xyz_full = numpy.matrix([[float(value) for value in first_list[b][0:3]] for b in first_stamps]).transpose()

    # 获取估计轨迹全部时间戳并排序
    second_stamps = second_list.keys()
    second_stamps.sort()
    # 估计轨迹完整3×N位置矩阵，乘用户scale
    second_xyz_full = numpy.matrix([[float(value)*float(args.scale) for value in second_list[b][0:3]] for b in second_stamps]).transpose()
    # 完整估计轨迹经过自动求解scale对齐之后的轨迹
    second_xyz_full_aligned = scale * rot * second_xyz_full + trans

    # verbose模式：打印全套误差指标
    if args.verbose:
        print "compared_pose_pairs %d pairs"%(len(trans_error))

        print "absolute_translational_error.rmse %f m"%numpy.sqrt(numpy.dot(trans_error,trans_error) / len(trans_error))
        print "absolute_translational_error.mean %f m"%numpy.mean(trans_error)
        print "absolute_translational_error.median %f m"%numpy.median(trans_error)
        print "absolute_translational_error.std %f m"%numpy.std(trans_error)
        print "absolute_translational_error.min %f m"%numpy.min(trans_error)
        print "absolute_translational_error.max %f m"%numpy.max(trans_error)
        print "max idx: %i" %numpy.argmax(trans_error)
    else:
        # print "%f, %f " % (numpy.sqrt(numpy.dot(trans_error,trans_error) / len(trans_error)),  scale)
        # print "%f,%f" % (numpy.sqrt(numpy.dot(trans_error,trans_error) / len(trans_error)),  scale)
        # 非verbose：输出 无尺度校正RMSE,自动求解scale,带尺度校正RMSE
        print "%f,%f,%f" % (numpy.sqrt(numpy.dot(trans_error,trans_error) / len(trans_error)), scale, numpy.sqrt(numpy.dot(trans_errorGT,trans_errorGT) / len(trans_errorGT)))
        # print "%f" % len(trans_error)
    # verbose2：同时输出有无尺度校正两套ATE‑RMSE
    if args.verbose2:
        print "compared_pose_pairs %d pairs"%(len(trans_error))
        print "absolute_translational_error.rmse %f m"%numpy.sqrt(numpy.dot(trans_error,trans_error) / len(trans_error))
        print "absolute_translational_errorGT.rmse %f m"%numpy.sqrt(numpy.dot(trans_errorGT,trans_errorGT) / len(trans_errorGT))

    # 如果开启save_associations，保存匹配真值‑对齐估计位姿对
    if args.save_associations:
        file = open(args.save_associations,"w")
        file.write("\n".join(["%f %f %f %f %f %f %f %f"%(a,x1,y1,z1,b,x2,y2,z2) for (a,b),(x1,y1,z1),(x2,y2,z2) in zip(matches,first_xyz.transpose().A,second_xyz_aligned.transpose().A)]))
        file.close()

    # 如果开启‑save，保存scale=1对齐后的完整估计轨迹
    if args.save:
        file = open(args.save,"w")
        file.write("\n".join(["%f "%stamp+" ".join(["%f"%d for d in line]) for stamp,line in zip(second_stamps,second_xyz_notscaled_full.transpose().A)]))
        file.close()

    # 如果开启‑plot，绘制轨迹对比图输出pdf
    if args.plot:
        import matplotlib
        matplotlib.use('Agg')          # 无GUI后端，服务器环境也可绘图
        import matplotlib.pyplot as plt
        import matplotlib.pylab as pylab
        from matplotlib.patches import Ellipse
        fig = plt.figure()
        ax = fig.add_subplot(111)
        # 绘制真值轨迹：黑色实线
        plot_traj(ax,first_stamps,first_xyz_full.transpose().A,'-',"black","ground truth")
        # 绘制对齐后的估计轨迹：蓝色实线
        plot_traj(ax,second_stamps,second_xyz_full_aligned.transpose().A,'-',"blue","estimated")
        label="difference"
        # 用红色短线连接每一对匹配真值点与估计点，直观显示误差
        for (a,b),(x1,y1,z1),(x2,y2,z2) in zip(matches,first_xyz.transpose().A,second_xyz_aligned.transpose().A):
            ax.plot([x1,x2],[y1,y2],'-',color="red",label=label)
            label=""

        ax.legend()

        ax.set_xlabel('x [m]')
        ax.set_ylabel('y [m]')
        plt.axis('equal')        # XY轴等比例，防止轨迹几何畸变
        plt.savefig(args.plot,format="pdf")
