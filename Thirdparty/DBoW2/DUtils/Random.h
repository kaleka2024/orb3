/*        * File: Random.h  * Project: DUtils library  * Author: Dorian Galvez-Lopez  * Date: April 2010, November 2011  * Description: manages pseudo‑random numbers  * License: see the LICENSE.txt file  *  */
// 头文件保护，现代编译器专用，防止头文件重复包含
#pragma once
// 传统宏形式头文件保护，兼容老旧编译器
#ifndef __D_RANDOM__
#define __D_RANDOM__

// C标准库rand()、srand()、RAND_MAX所需头文件
#include <cstdlib>
// std::vector容器头文件，用于无重复随机数生成器存储候选数值
#include <vector>

// DUtils工具库命名空间
namespace DUtils {

/// 生成伪随机数的工具函数集合
class Random {
public:
    // 前置声明：无重复随机整数生成器内部类
    class UnrepeatedRandomizer;

public:
    /**
     * @brief 使用当前系统时间设置C库rand随机数种子
     */
    static void SeedRand();

    /**
     * @brief 使用当前系统时间设置随机种子，仅第一次调用时生效
     * 多次调用本函数不会重复重置随机种子
     */
    static void SeedRandOnce();

    /**
     * @brief 使用用户传入的整数值设置rand随机数种子
     * @param seed 用户指定随机种子
     */
    static void SeedRand(int seed);

    /**
     * @brief 使用传入种子设置随机种子，仅第一次调用时生效
     * @param seed 用户指定随机种子
     */
    static void SeedRandOnce(int seed);

    /**
     * @brief 获取 [0, 1] 闭区间的随机浮点数，模板支持任意数值类型T
     * @return [0..1] 范围内T类型随机数值
     */
    template <class T>
    static T RandomValue(){
        // rand返回0~RAND_MAX，除以RAND_MAX归一化到0~1
        return (T)rand()/(T)RAND_MAX;
    }

    /**
     * @brief 获取 [min, max] 闭区间内T类型随机数
     * @param min 输出区间下限
     * @param max 输出区间上限
     * @return [min..max] 范围内T类型随机数值
     */
    template <class T>
    static T RandomValue(T min, T max){
        // [0,1]随机值缩放至区间长度，叠加min偏移得到最终随机数
        return Random::RandomValue<T>() * (max - min) + min;
    }

    /**
     * @brief 获取 [min, max] 闭区间随机int整数
     * @param min 整数区间下限
     * @param max 整数区间上限
     * @return [min..max] 区间随机整数
     */
    static int RandomInt(int min, int max);

    /**
     * @brief 生成服从高斯正态分布的随机数，Box‑Muller变换算法实现
     * @param mean 高斯分布均值
     * @param sigma 高斯分布标准差
     */
    template <class T>
    static T RandomGaussianValue(T mean, T sigma)
    {
        // Box‑Muller变换，把均匀分布随机数转为标准正态分布
        T x1, x2, w, y1;
        do {
            // 将[0,1]均匀随机数映射到[-1, +1]
            x1 = (T)2. * RandomValue<T>() - (T)1.;
            x2 = (T)2. * RandomValue<T>() - (T)1.;
            // 计算极坐标半径平方
            w = x1 * x1 + x2 * x2;
        // 拒绝采样：w >=1或者w等于0时重新采样，避免数学计算异常
        } while ( w >= (T)1. || w == (T)0. );

        // Box‑Muller公式计算变换系数
        w = sqrt( ((T)-2.0 * log( w ) ) / w );
        // 得到标准正态分布样本 N(0,1)
        y1 = x1 * w;

        // 平移+缩放，转为目标均值、标准差的高斯分布返回
        return( mean + y1 * sigma );
    }

private:
    /// 标记SeedRandOnce系列函数是否已经执行过播种，true代表已播种
    static bool m_already_seeded;

};

// ---------------------------------------------------------------------------

/// 生成不会重复的伪随机整数；取完所有数字后自动重新开始
class Random::UnrepeatedRandomizer {
public:

    /**
     * @brief 构造无重复随机生成器，输出范围[min, max]闭区间
     * @param min 随机数范围下限
     * @param max 随机数范围上限
     */
    UnrepeatedRandomizer(int min, int max);
    // 默认析构函数，无资源释放
    ~UnrepeatedRandomizer(){}

    /**
     * @brief 拷贝构造函数，复制另一个无重复随机生成器实例
     * @param rnd 源随机生成器对象
     */
    UnrepeatedRandomizer(const UnrepeatedRandomizer& rnd);

    /**
     * @brief 赋值运算符重载，拷贝另一个无重复随机生成器实例
     * @param rnd 源随机生成器对象
     * @return *this 当前对象引用，支持链式赋值
     */
    UnrepeatedRandomizer& operator=(const UnrepeatedRandomizer& rnd);

    /**
     * @brief 获取一个从未返回过的随机整数；全部数字耗尽会自动重置候选池
     * @return 不重复随机整数
     */
    int get();

    /**
     * @brief 判断是否已经取出区间内全部数字，候选池为空
     * @return true 代表所有数值都已经返回过；调用get会自动重建候选池
     */
    inline bool empty() const { return m_values.empty(); }

    /**
     * @brief 查询还剩余多少个未被取出的候选数值
     * @return 剩余可取出数字数量
     */
    inline unsigned int left() const { return m_values.size(); }

    /**
     * @brief 重置随机器，恢复初始完整候选池，如同刚构造完成
     */
    void reset();

protected:
    /**
     * @brief 创建可用候选数值vector，填充完整[min,max]整数集合
     */
    void createValues();

protected:
    /// 随机数输出区间最小值
    int m_min;
    /// 随机数输出区间最大值
    int m_max;

    /// 还未被取出的候选数值数组
    std::vector<int> m_values;
};

}

#endif
