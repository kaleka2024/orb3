/*        * File: Random.cpp  * Project: DUtils library  * Author: Dorian Galvez-Lopez  * Date: April 2010  * Description: manages pseudo‑random numbers  * License: see the LICENSE.txt file  *  */
// 引入随机数工具类头文件
#include "Random.h"
// 引入时间戳工具，用于获取当前系统时间做随机种子
#include "Timestamp.h"
// C标准库rand/srand函数头文件
#include <cstdlib>
using namespace std;

// 静态成员变量：标记全局随机数是否已经完成种子初始化，初始为false未播种
bool DUtils::Random::m_already_seeded = false;

/**
 * @brief 使用当前系统时间作为种子初始化C库rand随机数发生器
 */
void DUtils::Random::SeedRand(){
        // 定义时间戳对象
        Timestamp time;
        // 将时间戳设置为当前系统时刻
        time.setToCurrentTime();
        // 获取浮点格式时间，强转为无符号整数作为随机种子传入srand
        srand((unsigned)time.getFloatTime());
}

/**
 * @brief 只执行一次基于系统时间的随机种子播种，多次调用不会重复播种
 */
void DUtils::Random::SeedRandOnce() {
  // 判断尚未播种才执行
  if(!m_already_seeded)
  {
    // 调用时间种子播种函数
    DUtils::Random::SeedRand();
    // 标记已经完成播种，后续调用直接跳过
    m_already_seeded = true;
  }
}

/**
 * @brief 使用传入的指定整数值作为随机种子初始化rand
 * @param seed 用户给定的随机种子
 */
void DUtils::Random::SeedRand(int seed) {
        // 使用外部传入种子设置随机发生器
        srand(seed);
}

/**
 * @brief 使用指定种子做随机播种，整个程序生命周期仅播种一次
 * @param seed 用户给定的随机种子
 */
void DUtils::Random::SeedRandOnce(int seed) {
  // 未播种状态才进入逻辑
  if(!m_already_seeded)
  {
    // 使用入参种子初始化随机发生器
    DUtils::Random::SeedRand(seed);
    // 设置已播种标记，防止重复播种
    m_already_seeded = true;
  }
}

/**
 * @brief 生成[min, max]闭区间内的随机整数，包含两端边界
 * @param min 区间最小值
 * @param max 区间最大值
 * @return 区间内随机int整数
 */
int DUtils::Random::RandomInt(int min, int max){
        // 计算区间内整数总个数，闭区间所以+1
        int d = max - min + 1;
        // rand()返回[0,RAND_MAX]，归一化到[0,1)，乘以区间长度得到偏移，叠加min得到最终随机数
        return int(((double)rand()/((double)RAND_MAX + 1.0)) * d) + min;
}

// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------

/**
 * @brief 构造不可重复随机整数生成器，生成范围[min,max]
 * @param min 随机数范围下限
 * @param max 随机数范围上限
 */
DUtils::Random::UnrepeatedRandomizer::UnrepeatedRandomizer(int min, int max) {
  // 如果输入下限小于等于上限，直接保存
  if(min <= max)
  {
    m_min = min;
    m_max = max;
  }
  else
  {
    // 如果传入参数大小颠倒，交换修正上下限
    m_min = max;
    m_max = min;
  }

  // 根据上下限初始化全部候选数值数组
  createValues();
}

// ---------------------------------------------------------------------------

/**
 * @brief 拷贝构造函数，复制另一个不可重复随机生成器对象
 * @param rnd 源随机生成器实例
 */
DUtils::Random::UnrepeatedRandomizer::UnrepeatedRandomizer
  (const DUtils::Random::UnrepeatedRandomizer& rnd) {
  // 调用重载赋值运算符完成拷贝
  *this = rnd;
}

// ---------------------------------------------------------------------------

/**
 * @brief 获取一个不重复的随机整数；全部数字取完后自动重置候选池
 * @return 返回范围内随机未使用过的整数
 */
int DUtils::Random::UnrepeatedRandomizer::get() {
  // 如果候选数组为空，重新生成全部候选数字
  if(empty()) createValues();

  // 保证全局rand随机种子只播种一次
  DUtils::Random::SeedRandOnce();

  // 在候选数组下标范围 [0, size‑1] 取随机下标
  int k = DUtils::Random::RandomInt(0, m_values.size()-1);
  // 取出该下标对应的随机值作为返回结果
  int ret = m_values[k];
  // 将数组末尾元素覆盖到被取出的位置，实现O(1)删除，避免数组移位
  m_values[k] = m_values.back();
  // 弹出数组末尾，移除已经被取用过的数字，保证不会再次选出
  m_values.pop_back();

  return ret;
}

// ---------------------------------------------------------------------------

/**
 * @brief 创建完整候选数值池，填充 [m_min, m_max] 的全部整数
 */
void DUtils::Random::UnrepeatedRandomizer::createValues() {
  // 计算总数字数量，闭区间
  int n = m_max - m_min + 1;

  // 调整容器大小容纳全部候选数
  m_values.resize(n);
  // 循环填充连续整数，从m_min开始依次递增
  for(int i = 0; i < n; ++i) m_values[i] = m_min + i;
}

// ---------------------------------------------------------------------------

/**
 * @brief 重置随机器，恢复全部候选数字，允许再次取出全部数字
 */
void DUtils::Random::UnrepeatedRandomizer::reset() {
  // 如果当前候选数组长度不等于完整集合大小，则重建候选池
  if((int)m_values.size() != m_max - m_min + 1) createValues();
}

// ---------------------------------------------------------------------------

/**
 * @brief 赋值运算符重载，完成UnrepeatedRandomizer对象深拷贝
 * @param rnd 源对象引用
 * @return *this 返回当前对象引用，支持链式赋值
 */
DUtils::Random::UnrepeatedRandomizer&
DUtils::Random::UnrepeatedRandomizer::operator=
  (const DUtils::Random::UnrepeatedRandomizer& rnd) {
  // 防止自赋值：对象地址不一样才拷贝
  if(this != &rnd)
  {
    // 复制下限成员
    this->m_min = rnd.m_min;
    // 复制上限成员
    this->m_max = rnd.m_max;
    // 复制候选数值数组
    this->m_values = rnd.m_values;
  }
  // 返回本对象引用，支持 a=b=c 链式赋值语法
  return *this;
}

// ---------------------------------------------------------------------------
