/**
 * File: FORB.cpp
 * Date: June 2012
 * Author: Dorian Galvez-Lopez
 * Description: functions for ORB descriptors
 * License: see the LICENSE.txt file
 *
 * Distance function has been modified
 *
 */

// STL动态数组
#include <vector>
// STL字符串
#include <string>
// 字符串流，用于描述子序列化字符串编解码
#include <sstream>
// GCC固定宽度整数类型头文件 int32_t
#include <stdint-gcc.h>

// FORB类头文件，ORB描述子操作实现，继承FClass
#include "FORB.h"

// 使用std命名空间
using namespace std;

namespace DBoW2 {

// --------------------------------------------------------------------------
// ORB描述子字节长度，标准ORB为32字节(256bit)
const int FORB::L=32;

/**
 * @brief 计算一组ORB二进制描述子的均值描述子
 * @param descriptors 输入一批描述子指针集合
 * @param mean [out]输出得到的均值描述子cv::Mat，1×32 CV_8UC1
 * @note 二进制描述子取均值策略：每一位统计1的个数，过半则该bit置1，否则置0
 */
void FORB::meanValue(const std::vector<FORB::pDescriptor> &descriptors,
  FORB::TDescriptor &mean)
{
  // 输入描述子集合为空
  if(descriptors.empty())
  {
    mean.release();
    return;
  }
  // 只有1个描述子，直接克隆作为均值
  else if(descriptors.size() == 1)
  {
    mean = descriptors[0]->clone();
  }
  else
  {
    // ORB共32字节，每字节8bit，总256bit；sum数组统计每一个bit位为1的计数
    vector<int> sum(FORB::L * 8, 0);

    // 遍历全部输入描述子
    for(size_t i = 0; i < descriptors.size(); ++i)
    {
      const cv::Mat &d = *descriptors[i];
      // 获取描述子uchar数据指针
      const unsigned char *p = d.ptr<unsigned char>();

      // 遍历该描述子每一个字节
      for(int j = 0; j < d.cols; ++j, ++p)
      {
        // 依次检测字节内8个bit，对应bit为1则sum对应位置计数+1
        if(*p & (1 << 7)) ++sum[ j*8     ];
        if(*p & (1 << 6)) ++sum[ j*8 + 1 ];
        if(*p & (1 << 5)) ++sum[ j*8 + 2 ];
        if(*p & (1 << 4)) ++sum[ j*8 + 3 ];
        if(*p & (1 << 3)) ++sum[ j*8 + 4 ];
        if(*p & (1 << 2)) ++sum[ j*8 + 5 ];
        if(*p & (1 << 1)) ++sum[ j*8 + 6 ];
        if(*p & (1))      ++sum[ j*8 + 7 ];
      }
    }

    // 初始化均值描述子：1行32列，8位无符号字符
    mean = cv::Mat::zeros(1, FORB::L, CV_8U);
    unsigned char *p = mean.ptr<unsigned char>();

    // 半数阈值：超过一半样本该bit为1，则均值该bit置1
    const int N2 = (int)descriptors.size() / 2 + descriptors.size() % 2;
    for(size_t i = 0; i < sum.size(); ++i)
    {
      // 当前bit位1的计数大于等于半数，置1
      if(sum[i] >= N2)
      {
        // set bit 设置对应bit位
        *p |= 1 << (7 - (i % 8));
      }

      // 每处理完8个bit，移动到下一个字节
      if(i % 8 == 7) ++p;
    }
  }
}

// --------------------------------------------------------------------------
/**
 * @brief 计算两个ORB二进制描述子之间汉明距离
 * @param a 输入描述子A cv::Mat(1,32,CV_8U)
 * @param b 输入描述子B cv::Mat(1,32,CV_8U)
 * @return int 返回汉明距离，即不相同bit的总个数，0~256
 * @note 使用经典并行位运算快速统计异或后1的个数，避免循环逐bit判断
 */
int FORB::distance(const FORB::TDescriptor &a,
  const FORB::TDescriptor &b)
{
  // Bit set count operation from
  // http://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetParallel

  // 按32位整数读取描述子内存，32字节一共8个int32_t
  const int *pa = a.ptr<int32_t>();
  const int *pb = b.ptr<int32_t>();

  int dist=0;

  // 循环8次，每次处理32bit
  for(int i=0; i<8; i++, pa++, pb++)
  {
      // 异或：bit不同则该bit置1，相同置0
      unsigned  int v = *pa ^ *pb;
      // 并行统计bit1数量的位运算算法
      v = v - ((v >> 1) & 0x55555555);
      v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
      // 累加这32bit中bit=1总数到dist
      dist += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
  }

  return dist;
}

// --------------------------------------------------------------------------
/**
 * @brief 将ORB描述子序列化为字符串，方便保存
 * @param a 输入ORB描述子Mat
 * @return std::string 返回序列化字符串，每个字节数字空格隔开
 */
std::string FORB::toString(const FORB::TDescriptor &a)
{
  stringstream ss;
  const unsigned char *p = a.ptr<unsigned char>();

  // 遍历描述子每个字节，转为十进制数字写入字符串流
  for(int i = 0; i < a.cols; ++i, ++p)
  {
    ss << (int)*p << " ";
  }

  return ss.str();
}

// --------------------------------------------------------------------------
/**
 * @brief 从序列化字符串还原出ORB描述子cv::Mat
 * @param a [out]输出还原后的描述子Mat
 * @param s 输入序列化字符串
 */
void FORB::fromString(FORB::TDescriptor &a, const std::string &s)
{
  // 创建1×32 CV_8U的ORB描述子内存
  a.create(1, FORB::L, CV_8U);
  unsigned char *p = a.ptr<unsigned char>();

  stringstream ss(s);
  // 循环读取32个字节数值
  for(int i = 0; i < FORB::L; ++i, ++p)
  {
    int n;
    ss >> n;

    // 读取没有出错则赋值
    if(!ss.fail())
     *p = (unsigned char)n;
  }

}

// --------------------------------------------------------------------------
/**
 * @brief 将一批ORB二进制描述子转换为浮点矩阵，每个bit展开成0/1 float
 * @param descriptors 输入ORB描述子vector，每个元素1×32 CV_8U
 * @param mat [out]输出N × 256 CV_32F矩阵，每一行对应一个描述子256个bit(0.0f或1.0f)
 */
void FORB::toMat32F(const std::vector<TDescriptor> &descriptors,
  cv::Mat &mat)
{
  if(descriptors.empty())
  {
    mat.release();
    return;
  }

  const size_t N = descriptors.size();

  // N行，每行256维float
  mat.create(N, FORB::L*8, CV_32F);
  float *p = mat.ptr<float>();

  // 遍历每一个描述子
  for(size_t i = 0; i < N; ++i)
  {
    const int C = descriptors[i].cols;
    const unsigned char *desc = descriptors[i].ptr<unsigned char>();

    // 遍历每个字节，把8个bit展开为8个float
    for(int j = 0; j < C; ++j, p += 8)
    {
      p[0] = (desc[j] & (1 << 7) ? 1 : 0);
      p[1] = (desc[j] & (1 << 6) ? 1 : 0);
      p[2] = (desc[j] & (1 << 5) ? 1 : 0);
      p[3] = (desc[j] & (1 << 4) ? 1 : 0);
      p[4] = (desc[j] & (1 << 3) ? 1 : 0);
      p[5] = (desc[j] & (1 << 2) ? 1 : 0);
      p[6] = (desc[j] & (1 << 1) ? 1 : 0);
      p[7] = desc[j] & (1);
    }
  }
}

// --------------------------------------------------------------------------
/**
 * @brief 将一批ORB描述子直接拷贝输出为8U矩阵，每行一个原始32字节ORB描述子
 * @param descriptors 输入ORB描述子数组
 * @param mat [out]输出 N × 32 CV_8U矩阵
 */
void FORB::toMat8U(const std::vector<TDescriptor> &descriptors,
  cv::Mat &mat)
{
  mat.create(descriptors.size(), 32, CV_8U);

  unsigned char *p = mat.ptr<unsigned char>();

  // 逐个拷贝原始32字节数据
  for(size_t i = 0; i < descriptors.size(); ++i, p += 32)
  {
    const unsigned char *d = descriptors[i].ptr<unsigned char>();
    std::copy(d, d+32, p);
  }

}

// --------------------------------------------------------------------------
} // namespace DBoW2
