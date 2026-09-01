#!/usr/bin/python
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
The Kinect provides the color and depth images in an un-synchronized way. This means that the set of time stamps from the color images do not intersect with those of the depth images. Therefore, we need some way of associating color images to depth images.

For this purpose, you can use the ''associate.py'' script. It reads the time stamps from the rgb.txt file and the depth.txt file, and joins them by finding the best matches.
"""

# 导入命令行参数解析模块，用于解析脚本运行时传入的参数
import argparse


def read_file_list(filename, remove_bounds):
    """
    Reads a trajectory from a text file.

    File format:
    The file format is "stamp d1 d2 d3 ...", where stamp denotes the time stamp (to be matched)
    and "d1 d2 d3.." is arbitary data (e.g., a 3D position and 3D orientation) associated to this timestamp.

    Input:
    filename -- File name

    Output:
    dict -- dictionary of (stamp,data) tuples
    """
    # 以只读模式打开目标时间戳文本文件
    file = open(filename)
    # 一次性读取文件全部内容到字符串data
    data = file.read()
    # 将逗号、制表符全部替换为空格，再按换行符切割，得到所有行字符串列表
    lines = data.replace(",", " ").replace("\t", " ").split("\n")
    # 如果开启remove_bounds，丢弃文件开头100行、末尾100行，过滤掉头尾无效数据
    if remove_bounds:
        lines = lines[100:-100]
    # 列表推导式逐行处理：按空格分割，剔除空字符串；跳过空行与#开头注释行
    list = [
        [v.strip() for v in line.split(" ") if v.strip() != ""]
        for line in lines
        if len(line) > 0 and line[0] != "#"
    ]
    # 将每一行第0个元素转为浮点时间戳，后面元素作为附属数据；过滤字段数不足2的无效行
    list = [(float(l[0]), l[1:]) for l in list if len(l) > 1]
    # 返回字典：key=时间戳(float)，value=该行其余字段组成的列表
    return dict(list)


def associate(first_list, second_list, offset, max_difference):
    """
    Associate two dictionaries of (stamp,data). As the time stamps never match exactly, we aim
    to find the closest match for every input tuple.

    Input:
    first_list -- first dictionary of (stamp,data) tuples
    second_list -- second dictionary of (stamp,data) tuples
    offset -- time offset between both dictionaries (e.g., to model the delay between the sensors)
    max_difference -- search radius for candidate generation

    Output:
    matches -- list of matched tuples ((stamp1,data1),(stamp2,data2))
    """
    # 获取第一个字典全部时间戳key集合
    first_keys = first_list.keys()
    # 获取第二个字典全部时间戳key集合
    second_keys = second_list.keys()
    # 双重循环遍历两组所有时间戳；计算加上offset后的时间差，只保留差值小于max_difference的候选对
    # 元组内容：(时间差, 第一组时间戳a, 第二组时间戳b)
    potential_matches = [
        (abs(a - (b + offset)), a, b)
        for a in first_keys
        for b in second_keys
        if abs(a - (b + offset)) < max_difference
    ]
    # 将候选匹配列表按时间差从小到大排序，优先取时间差最小的配对
    potential_matches.sort()
    # 用于存放最终成功配对的时间戳对
    matches = []
    # 遍历排序后的候选匹配
    for diff, a, b in potential_matches:
        # 校验a、b仍然未被使用（还在key集合内），避免一个时间戳被多次匹配
        if a in first_keys and b in second_keys:
            # 从key集合移除a，标记该时间戳已经配对，不再参与后续匹配
            first_keys.remove(a)
            # 从key集合移除b，标记该时间戳已经配对，不再参与后续匹配
            second_keys.remove(b)
            # 将(a,b)时间戳对加入匹配结果列表
            matches.append((a, b))
    # 将最终匹配结果按照第一路时间戳升序排序
    matches.sort()
    return matches


if __name__ == "__main__":
    # parse command line
    # 创建命令行参数解析器对象，添加脚本描述文本
    parser = argparse.ArgumentParser(
        description="""
    This script takes two data files with timestamps and associates them
    """
    )
    # 位置参数：第一个带时间戳的文本文件
    parser.add_argument("first_file", help="first text file (format: timestamp data)")
    # 位置参数：第二个带时间戳的文本文件
    parser.add_argument("second_file", help="second text file (format: timestamp data)")
    # 可选布尔参数--first_only：仅输出第一个文件的匹配行，不输出第二个文件内容
    parser.add_argument(
        "--first_only",
        help="only output associated lines from first file",
        action="store_true",
    )
    # 可选参数--offset：给第二个文件时间戳叠加的时间偏移量，模拟传感器之间时间延迟，默认0.0
    parser.add_argument(
        "--offset",
        help="time offset added to the timestamps of the second file (default: 0.0)",
        default=0.0,
    )
    # 可选参数--max_difference：允许配对的最大时间差阈值，单位秒，默认0.02秒
    parser.add_argument(
        "--max_difference",
        help="maximally allowed time difference for matching entries (default: 0.02)",
        default=0.02,
    )
    # 解析命令行参数，结果存入args对象
    args = parser.parse_args()

    # 读取第一个文件，remove_bounds不传，使用默认None
    first_list = read_file_list(args.first_file, None)
    # 读取第二个文件，remove_bounds不传，使用默认None
    second_list = read_file_list(args.second_file, None)

    # 执行时间戳关联匹配，offset与max_difference转为浮点数传入
    matches = associate(
        first_list, second_list, float(args.offset), float(args.max_difference)
    )

    # 判断是否开启--first_only，只输出第一个文件匹配项
    if args.first_only:
        for a, b in matches:
            # 打印：第一个文件时间戳 + 该行附属数据
            print("%f %s" % (a, " ".join(first_list[a])))
    else:
        # 未开启--first_only，两路信息全部输出
        for a, b in matches:
            print(
                "%f %s %f %s"
                % (
                    a,
                    " ".join(first_list[a]),
                    b - float(args.offset),
                    " ".join(second_list[b]),
                )
            )
