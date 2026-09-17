///
// expected - An implementation of std::expected with extensions
// Written in 2017 by Sy Brand (tartanllama@gmail.com, @TartanLlama)
//
// Documentation available at http://tl.tartanllama.xyz/
//
// To the extent possible under law, the author(s) have dedicated all
// copyright and related and neighboring rights to this software to the
// public domain worldwide. This software is distributed without any warranty.
//
// You should have received a copy of the CC0 Public Domain Dedication
// along with this software. If not, see
// <http://creativecommons.org/publicdomain/zero/1.0/>.
///

/*===========================================================================
 * 文件名称：expected.hpp —— tl::expected<T,E> 的单头实现（std::expected 的扩展版，v1.3.1）
 * 作者：Sy Brand(TartanLlama)，CC0 公有领域。
 * 核心思想：
 *   expected<T,E> 要么持有一个“成功值 T”，要么持有一个“错误值 E”，用 m_has_val 区分；
 *   它是返回值风格的错误处理（不依赖异常），并提供 and_then/map/map_error/or_else 等单子接口。
 * 实现骨架（自底向上的多层继承，目的是按 T/E 的类型性质条件性地得到“平凡”特殊成员）：
 *   expected_storage_base   —— 联合体存储 T 或 unexpected<E>，按平凡析构性做 6 个特化；
 *   expected_operations_base—— placement-new 构造/销毁、强异常安全的 assign、get/geterr；
 *   expected_copy/move/copy_assign/move_assign_base —— 条件平凡的拷贝/移动；
 *   expected_delete_*_base  —— 按可构造/可赋值性 delete 对应特殊成员；
 *   expected_default_ctor_base —— T 不可默认构造时删除默认构造；
 *   最外层 expected<T,E> 私有继承上述基类，对外暴露观察者与单子 API。
 * 阅读提示：本文件大量使用 SFINAE(enable_if)、标签分派与“按特性特化”，重复模板较多，
 *           抓住“存储层 -> 操作层 -> 特殊成员控制层 -> 对外 expected”这条主线即可。
 *===========================================================================*/
#ifndef TL_EXPECTED_HPP
#define TL_EXPECTED_HPP

#define TL_EXPECTED_VERSION_MAJOR 1  // 主版本号
#define TL_EXPECTED_VERSION_MINOR 3  // 次版本号
#define TL_EXPECTED_VERSION_PATCH 1  // 补丁版本号

#include <exception>  // std::exception：bad_expected_access 的基类
#include <functional>  // std::mem_fn：C++11 回填 invoke 时使用
#include <type_traits>  // 类型萃取：is_constructible/enable_if 等 SFINAE 基础设施
#include <utility>  // std::forward/move/declval/addressof 等

// ---- 异常特性检测：GCC/Clang 定义 __EXCEPTIONS，MSVC 定义 _CPPUNWIND ----
#if defined(__EXCEPTIONS) || defined(_CPPUNWIND)
#define TL_EXPECTED_EXCEPTIONS_ENABLED  // 二者之一存在 => 编译期开启了异常，后续可用 throw/try
#endif

// ---- 识别 MSVC2015(_MSC_VER==1900)：该版本 constexpr 支持残缺，需把部分 constexpr 置空 ----
#if (defined(_MSC_VER) && _MSC_VER == 1900)
#define TL_EXPECTED_MSVC2015  // 标记当前为 MSVC2015
#define TL_EXPECTED_MSVC2015_CONSTEXPR  // MSVC2015 下该宏为空(不写 constexpr)
// ---- 禁用异常时：没有回滚负担，直接拷贝/移动构造即可 ----
#else
#define TL_EXPECTED_MSVC2015_CONSTEXPR constexpr  // 其它编译器正常使用 constexpr
#endif

// ---- 识别 GCC4.9：不支持成员函数 const&& 重载、部分 C++11 traits 有缺陷 ----
#if (defined(__GNUC__) && __GNUC__ == 4 && __GNUC_MINOR__ <= 9 &&              \
     !defined(__clang__))
#define TL_EXPECTED_GCC49  // 标记 GCC4.9
#endif

// ---- 识别 GCC5.4 及以下 ----
#if (defined(__GNUC__) && __GNUC__ == 5 && __GNUC_MINOR__ <= 4 &&              \
     !defined(__clang__))
#define TL_EXPECTED_GCC54  // 标记 GCC5.4
#endif

// ---- 识别 GCC5.5 及以下 ----
#if (defined(__GNUC__) && __GNUC__ == 5 && __GNUC_MINOR__ <= 5 &&              \
     !defined(__clang__))
#define TL_EXPECTED_GCC55  // 标记 GCC5.5
#endif

// ---- 统一“当前 C++ 标准版本”宏：MSVC 用 _MSVC_LANG，其它用 __cplusplus ----
#ifdef _MSVC_LANG
#define TL_CPLUSPLUS _MSVC_LANG  // MSVC 取 _MSVC_LANG
#else
#define TL_CPLUSPLUS __cplusplus  // 标准实现取 __cplusplus
#endif

// ---- 断言宏 TL_ASSERT：允许用户在外部预先定义以替换默认 assert ----
// C++11 的 constexpr 函数里不能用 assert，且 GCC4.9 有编译器 bug，故这些情况下置空
#if !defined(TL_ASSERT)
//can't have assert in constexpr in C++11 and GCC 4.9 has a compiler bug
#if (TL_CPLUSPLUS > 201103L) && !defined(TL_EXPECTED_GCC49)
#include <cassert>  // C++14 及以上且非 GCC4.9 才包含 <cassert>
#define TL_ASSERT(x) assert(x)  // 正常情况：断言就是标准 assert
#else
#define TL_ASSERT(x)  // 受限情况：断言退化为空(不检查)
#endif
#endif

#if (defined(__GNUC__) && __GNUC__ == 4 && __GNUC_MINOR__ <= 9 &&              \
     !defined(__clang__))
// GCC < 5 doesn't support overloading on const&& for member functions

#define TL_EXPECTED_NO_CONSTRR  // GCC<5 不支持 const&& 成员重载，用此宏屏蔽相关代码
// GCC < 5 doesn't support some standard C++11 type traits
// 下面三个宏把“平凡拷贝构造/赋值/析构”判断封装起来，以兼容老 GCC 残缺的 traits
#define TL_EXPECTED_IS_TRIVIALLY_COPY_CONSTRUCTIBLE(T)                         \
  std::has_trivial_copy_constructor<T>  // GCC4.9 用旧的 has_trivial_*
#define TL_EXPECTED_IS_TRIVIALLY_COPY_ASSIGNABLE(T)                            \
  std::has_trivial_copy_assign<T>

// This one will be different for GCC 5.7 if it's ever supported
#define TL_EXPECTED_IS_TRIVIALLY_DESTRUCTIBLE(T)                               \
  std::is_trivially_destructible<T>

// GCC 5 < v < 8 has a bug in is_trivially_copy_constructible which breaks
// std::vector for non-copyable types
#elif (defined(__GNUC__) && __GNUC__ < 8 && !defined(__clang__))
// ---- GCC5~7：is_trivially_copy_constructible 对不可拷贝类型会让 std::vector 出错，
//      故在 detail 里包一层，并对 std::vector 特化为 false_type（include 守卫防重复定义）----
#ifndef TL_GCC_LESS_8_TRIVIALLY_COPY_CONSTRUCTIBLE_MUTEX
#define TL_GCC_LESS_8_TRIVIALLY_COPY_CONSTRUCTIBLE_MUTEX
/*===========================================================================
 * 第一部分：前置声明、unexpected<E>(错误值包装)、in_place/unexpect 标签
 *===========================================================================*/
namespace tl {  // 进入 tl 命名空间
/*===========================================================================
 * 第二部分：detail —— 为兼容 C++11 而回填的 C++14/17 工具(traits/invoke/swappable)
 * 以及 expected 专用的 SFINAE 别名
 *===========================================================================*/
namespace detail {
template <class T>
struct is_trivially_copy_constructible  // 默认转发到标准 traits
    : std::is_trivially_copy_constructible<T> {};
#ifdef _GLIBCXX_VECTOR
template <class T, class A>
struct is_trivially_copy_constructible<std::vector<T, A>> : std::false_type {};  // 仅当确实使用了 libstdc++ 的 vector(_GLIBCXX_VECTOR)时，对其特化为非平凡
#endif
} // namespace detail
} // namespace tl  // tl 命名空间结束
#endif

#define TL_EXPECTED_IS_TRIVIALLY_COPY_CONSTRUCTIBLE(T)                         \
  tl::detail::is_trivially_copy_constructible<T>
#define TL_EXPECTED_IS_TRIVIALLY_COPY_ASSIGNABLE(T)                            \
  std::is_trivially_copy_assignable<T>
#define TL_EXPECTED_IS_TRIVIALLY_DESTRUCTIBLE(T)                               \
  std::is_trivially_destructible<T>
#else
#define TL_EXPECTED_IS_TRIVIALLY_COPY_CONSTRUCTIBLE(T)                         \
  std::is_trivially_copy_constructible<T>
#define TL_EXPECTED_IS_TRIVIALLY_COPY_ASSIGNABLE(T)                            \
  std::is_trivially_copy_assignable<T>
#define TL_EXPECTED_IS_TRIVIALLY_DESTRUCTIBLE(T)                               \
  std::is_trivially_destructible<T>
#endif

// ---- 是否启用 C++14(返回值推导等) ----
#if TL_CPLUSPLUS > 201103L
#define TL_EXPECTED_CXX14  // 标准高于 C++11 即定义 CXX14 宏
#endif

// ---- GCC4.9 下 constexpr 置空的专用宏 ----
#ifdef TL_EXPECTED_GCC49
#define TL_EXPECTED_GCC49_CONSTEXPR  // GCC4.9：空
#else
#define TL_EXPECTED_GCC49_CONSTEXPR constexpr  // 否则：constexpr
#endif

// ---- 11_CONSTEXPR：仅在“真正支持的 C++ 版本”上给函数加 constexpr，C++11/MSVC2015/GCC4.9 置空 ----
#if (TL_CPLUSPLUS == 201103L || defined(TL_EXPECTED_MSVC2015) ||                \
     defined(TL_EXPECTED_GCC49))
#define TL_EXPECTED_11_CONSTEXPR  // 受限环境：空
#else
#define TL_EXPECTED_11_CONSTEXPR constexpr  // 正常环境：constexpr
#endif

// ---- C++17 才有 [[nodiscard]]，低版本置空 ----
#if TL_CPLUSPLUS >= 201703L
#define TL_EXPECTED_NODISCARD [[nodiscard]]  // C++17：标记结果不可忽略
#else
#define TL_EXPECTED_NODISCARD  // 低版本：空
#endif

namespace tl {
template <class T, class E>
class TL_EXPECTED_NODISCARD expected;  // 前置声明 expected 模板，供 traits 识别使用

// ---- monostate(空类型)与 in_place_t(就地构造标签)，带防重复包含守卫 ----
#ifndef TL_MONOSTATE_INPLACE_MUTEX
#define TL_MONOSTATE_INPLACE_MUTEX
class monostate {};  // monostate：无任何数据的单元类型，用于 map_error 返回 void 错误时占位

struct in_place_t {  // in_place_t：标签类型，表示“就地构造成功值 T”
  explicit in_place_t() = default;
};
static constexpr in_place_t in_place{};  // 其全局标签实例 in_place
#endif

template <class E>
/*---------------------------------------------------------------------------
 * unexpected<E>：错误值 E 的轻量包装。expected 出错时实际存放的就是它。
 * 禁止默认构造；支持拷贝/移动/原地/初始化列表构造；按左右值限定符提供 value()。
 *---------------------------------------------------------------------------*/
class unexpected {
public:
  static_assert(!std::is_same<E, void>::value, "E must not be void");  // 错误类型 E 不允许是 void

  unexpected() = delete;  // 删除默认构造：必须给出一个错误值
  constexpr explicit unexpected(const E &e) : m_val(e) {}  // 由 const 左值拷贝构造错误值

  constexpr explicit unexpected(E &&e) : m_val(std::move(e)) {}  // 由右值移动构造错误值

  template <class... Args, typename std::enable_if<std::is_constructible<
                               E, Args &&...>::value>::type * = nullptr>
  constexpr explicit unexpected(Args &&...args)  // 可变参原地构造：用 Args... 直接构造 E(SFINAE 要求可构造)
      : m_val(std::forward<Args>(args)...) {}
  template <
      class U, class... Args,
      typename std::enable_if<std::is_constructible<
          E, std::initializer_list<U> &, Args &&...>::value>::type * = nullptr>
  constexpr explicit unexpected(std::initializer_list<U> l, Args &&...args)  // 初始化列表 + 额外参数原地构造 E
      : m_val(l, std::forward<Args>(args)...) {}

  constexpr const E &value() const & { return m_val; }  // const 左值：取 const 引用
  TL_EXPECTED_11_CONSTEXPR E &value() & { return m_val; }  // 非 const 左值：取可写引用
  TL_EXPECTED_11_CONSTEXPR E &&value() && { return std::move(m_val); }  // 右值：移动取出
  constexpr const E &&value() const && { return std::move(m_val); }  // const 右值：移动取出

private:
  E m_val;  // 被包装的错误值(唯一数据成员)
};

// ---- C++17 类模板参数推导(CTAD)：unexpected(e) 自动推出 unexpected<E> ----
#ifdef __cpp_deduction_guides
template <class E>
unexpected(E) -> unexpected<E>;  // 推导指引
#endif

template <class E>
// unexpected 的六个关系运算符，全部委托给 E 的对应运算符
constexpr bool operator==(const unexpected<E> &lhs, const unexpected<E> &rhs) {  // 相等：比较内部错误值
  return lhs.value() == rhs.value();
}
template <class E>
constexpr bool operator!=(const unexpected<E> &lhs, const unexpected<E> &rhs) {  // 不等
  return lhs.value() != rhs.value();
}
template <class E>
constexpr bool operator<(const unexpected<E> &lhs, const unexpected<E> &rhs) {  // 小于
  return lhs.value() < rhs.value();
}
template <class E>
constexpr bool operator<=(const unexpected<E> &lhs, const unexpected<E> &rhs) {  // 小于等于
  return lhs.value() <= rhs.value();
}
template <class E>
constexpr bool operator>(const unexpected<E> &lhs, const unexpected<E> &rhs) {  // 大于
  return lhs.value() > rhs.value();
}
template <class E>
constexpr bool operator>=(const unexpected<E> &lhs, const unexpected<E> &rhs) {  // 大于等于
  return lhs.value() >= rhs.value();
}

template <class E>
// 工厂函数：剥掉引用/cv 后包装成 unexpected（类似 std::make_*）
unexpected<typename std::decay<E>::type> make_unexpected(E &&e) {  // 返回 unexpected<decay(E)>
  return unexpected<typename std::decay<E>::type>(std::forward<E>(e));
}

struct unexpect_t {  // unexpect_t：标签类型，表示“构造的是错误值”(与 in_place_t 相对)
  unexpect_t() = default;
};
static constexpr unexpect_t unexpect{};  // 全局标签实例 unexpect

// ---- 抛异常统一入口：开启异常时 throw；禁用异常时直接 std::terminate 终止 ----
#ifdef TL_EXPECTED_EXCEPTIONS_ENABLED
#define TL_EXPECTED_THROW_EXCEPTION(e) throw((e));  // 有异常：抛出
#else
#define TL_EXPECTED_THROW_EXCEPTION(e) std::terminate();  // 无异常：终止进程
#endif
namespace detail {
#ifndef TL_TRAITS_MUTEX  // traits 块防重复定义守卫
#define TL_TRAITS_MUTEX
// C++14-style aliases for brevity
template <class T>
using remove_const_t = typename std::remove_const<T>::type;  // C++14 _t 别名回填：去 const
template <class T>
using remove_reference_t = typename std::remove_reference<T>::type;  // 去引用
template <class T>
using decay_t = typename std::decay<T>::type;  // 退化(去引用/cv/数组函数转换)
template <bool E, class T = void>
using enable_if_t = typename std::enable_if<E, T>::type;  // SFINAE 用 enable_if _t
template <bool B, class T, class F>
using conditional_t = typename std::conditional<B, T, F>::type;  // 编译期选择 _t

// std::conjunction from C++17
template <class...>
// C++17 std::conjunction 的 C++11 递归实现：逻辑与(短路)
struct conjunction : std::true_type {};  // 空包 => true
template <class B>
struct conjunction<B> : B {};  // 单参数 => 自身
template <class B, class... Bs>
struct conjunction<B, Bs...>  // 首项为真则继续递归，否则直接取首项(短路)
    : std::conditional<bool(B::value), conjunction<Bs...>, B>::type {};

// ---- libc++ 在 C++11 下 std::mem_fn 用在 noexcept 表达式会硬报错，准备一组绕过 traits ----
#if defined(_LIBCPP_VERSION) && __cplusplus == 201103L
#define TL_TRAITS_LIBCXX_MEM_FN_WORKAROUND
#endif

// In C++11 mode, there's an issue in libc++'s std::mem_fn
// which results in a hard-error when using it in a noexcept expression
// in some cases. This is a check to workaround the common failing case.
#ifdef TL_TRAITS_LIBCXX_MEM_FN_WORKAROUND
template <class T>
struct is_pointer_to_non_const_member_func : std::false_type {};  // 基模板：不是“非 const 成员函数指针”
template <class T, class Ret, class... Args>
struct is_pointer_to_non_const_member_func<Ret (T::*)(Args...)>  // 匹配普通成员函数指针(及其 &/&&/volatile 变体)
    : std::true_type {};
template <class T, class Ret, class... Args>
struct is_pointer_to_non_const_member_func<Ret (T::*)(Args...) &>
    : std::true_type {};
template <class T, class Ret, class... Args>
struct is_pointer_to_non_const_member_func<Ret (T::*)(Args...) &&>
    : std::true_type {};
template <class T, class Ret, class... Args>
struct is_pointer_to_non_const_member_func<Ret (T::*)(Args...) volatile>
    : std::true_type {};
template <class T, class Ret, class... Args>
struct is_pointer_to_non_const_member_func<Ret (T::*)(Args...) volatile &>
    : std::true_type {};
template <class T, class Ret, class... Args>
struct is_pointer_to_non_const_member_func<Ret (T::*)(Args...) volatile &&>
    : std::true_type {};

template <class T>
struct is_const_or_const_ref : std::false_type {};
template <class T>
struct is_const_or_const_ref<T const &> : std::true_type {};  // 是否 const 或 const&(两个偏特化)
template <class T>
struct is_const_or_const_ref<T const> : std::true_type {};
#endif

// std::invoke from C++17
// https://stackoverflow.com/questions/38288042/c11-14-invoke-workaround
template <
    typename Fn, typename... Args,
#ifdef TL_TRAITS_LIBCXX_MEM_FN_WORKAROUND
    typename = enable_if_t<!(is_pointer_to_non_const_member_func<Fn>::value &&
                             is_const_or_const_ref<Args...>::value)>,
#endif
    typename = enable_if_t<std::is_member_pointer<decay_t<Fn>>::value>, int = 0>
// C++17 std::invoke 的 C++11 回填：第一个重载处理“成员指针”(用 mem_fn 调用)
constexpr auto invoke(Fn &&f, Args &&...args) noexcept(  // 成员指针版本：返回类型与 noexcept 都由 mem_fn 推导
    noexcept(std::mem_fn(f)(std::forward<Args>(args)...)))
    -> decltype(std::mem_fn(f)(std::forward<Args>(args)...)) {
  return std::mem_fn(f)(std::forward<Args>(args)...);  // 转发给 mem_fn 完成调用
}

template <typename Fn, typename... Args,
          typename = enable_if_t<!std::is_member_pointer<decay_t<Fn>>::value>>
// invoke 第二个重载：普通可调用对象(非成员指针)，直接完美转发调用
constexpr auto invoke(Fn &&f, Args &&...args) noexcept(  // 普通可调用对象版本
    noexcept(std::forward<Fn>(f)(std::forward<Args>(args)...)))
    -> decltype(std::forward<Fn>(f)(std::forward<Args>(args)...)) {
  return std::forward<Fn>(f)(std::forward<Args>(args)...);  // 完美转发调用
}

// std::invoke_result from C++17
template <class F, class, class... Us>
// C++17 std::invoke_result：用 decltype(invoke(...)) 求调用结果类型
struct invoke_result_impl;

template <class F, class... Us>
struct invoke_result_impl<
    F,
    decltype(detail::invoke(std::declval<F>(), std::declval<Us>()...), void()),
    Us...> {
  using type =
      decltype(detail::invoke(std::declval<F>(), std::declval<Us>()...));
};

template <class F, class... Us>
using invoke_result = invoke_result_impl<F, void, Us...>;  // 对外别名(中间的 void 仅用于偏特化匹配)

template <class F, class... Us>
using invoke_result_t = typename invoke_result<F, Us...>::type;  // 结果类型 _t 简写

// ---- is_swappable/is_nothrow_swappable：MSVC2015 无法正确实现，一律乐观地视为可 swap ----
#if defined(_MSC_VER) && _MSC_VER <= 1900
// TODO make a version which works with MSVC 2015
template <class T, class U = T>
struct is_swappable : std::true_type {};  // MSVC2015：直接认为可交换

template <class T, class U = T>
struct is_nothrow_swappable : std::true_type {};  // MSVC2015：直接认为 noexcept 可交换
#else
// https://stackoverflow.com/questions/26744589/what-is-a-proper-way-to-implement-is-swappable-to-test-for-the-swappable-concept
// 其它编译器：借助 ADL(实参依赖查找)机制在编译期探测 swap 是否合法、是否 noexcept。
// 思路：声明返回 tag 的 swap 模板，再用 decltype 判断非限定 swap 调用解析到谁。
namespace swap_adl_tests {
// if swap ADL finds this then it would call std::swap otherwise (same
// signature)
struct tag {};  // 探测用标记类型

template <class T>
tag swap(T &, T &);  // 声明(不定义)通用 swap，数组也声明一个
template <class T, std::size_t N>
tag swap(T (&a)[N], T (&b)[N]);

// helper functions to test if an unqualified swap is possible, and if it
// becomes std::swap
template <class, class>
std::false_type can_swap(...) noexcept(false);  // 兜底重载：任意类型 => 不可 swap(可变参优先级最低)
template <class T, class U,
          class = decltype(swap(std::declval<T &>(), std::declval<U &>()))>
std::true_type can_swap(int) noexcept(noexcept(swap(std::declval<T &>(),  // 优先重载：若 swap(T&,U&) 合法 => 可 swap，并透传 noexcept
                                                    std::declval<U &>())));

template <class, class>
std::false_type uses_std(...);  // 兜底：未解析到 tag 版本
template <class T, class U>
std::is_same<decltype(swap(std::declval<T &>(), std::declval<U &>())), tag>
uses_std(int);  // 若 swap 解析为上面声明的 tag 版本，说明最终会落到 std::swap

template <class T>
struct is_std_swap_noexcept  // std::swap 的 noexcept 等价于“可 noexcept 移动构造 + 可 noexcept 移动赋值”
    : std::integral_constant<bool,
                             std::is_nothrow_move_constructible<T>::value &&
                                 std::is_nothrow_move_assignable<T>::value> {};

template <class T, std::size_t N>
struct is_std_swap_noexcept<T[N]> : is_std_swap_noexcept<T> {};  // 数组沿用元素类型结论

template <class T, class U>
struct is_adl_swap_noexcept  // ADL 自定义 swap 的 noexcept 直接取 can_swap 的 noexcept
    : std::integral_constant<bool, noexcept(can_swap<T, U>(0))> {};
} // namespace swap_adl_tests

template <class T, class U = T>
// 汇总：可 swap = 能解析到 swap，且(用的是自定义 swap 或 T 可移动构造/赋值)
struct is_swappable
    : std::integral_constant<
          bool,
          decltype(detail::swap_adl_tests::can_swap<T, U>(0))::value &&
              (!decltype(detail::swap_adl_tests::uses_std<T, U>(0))::value ||
               (std::is_move_assignable<T>::value &&
                std::is_move_constructible<T>::value))> {};

template <class T, std::size_t N>
struct is_swappable<T[N], T[N]>  // 数组版本：元素可交换则数组可交换
    : std::integral_constant<
          bool,
          decltype(detail::swap_adl_tests::can_swap<T[N], T[N]>(0))::value &&
              (!decltype(detail::swap_adl_tests::uses_std<T[N], T[N]>(
                   0))::value ||
               is_swappable<T, T>::value)> {};

template <class T, class U = T>
// 汇总 noexcept 可交换：先可交换，再按 std::swap / ADL swap 分别判定
struct is_nothrow_swappable
    : std::integral_constant<
          bool,
          is_swappable<T, U>::value &&
              ((decltype(detail::swap_adl_tests::uses_std<T, U>(0))::value &&
                detail::swap_adl_tests::is_std_swap_noexcept<T>::value) ||
               (!decltype(detail::swap_adl_tests::uses_std<T, U>(0))::value &&
                detail::swap_adl_tests::is_adl_swap_noexcept<T, U>::value))> {};
#endif
#endif

// Trait for checking if a type is a tl::expected
template <class T>
// ---- 判断一个类型是不是 tl::expected 的特征类 ----
struct is_expected_impl : std::false_type {};  // 基模板：不是 expected
template <class T, class E>
struct is_expected_impl<expected<T, E>> : std::true_type {};  // 偏特化命中 expected<T,E> => 真
template <class T>
using is_expected = is_expected_impl<decay_t<T>>;  // 退化后再判断

template <class T, class E, class U>
// SFINAE：允许把一个 U 当作“成功值”构造/赋值给 expected 的条件：
// U 能构造 T，且 U 不是 in_place_t、不是 expected 自身、不是 unexpected。
using expected_enable_forward_value = detail::enable_if_t<
    std::is_constructible<T, U &&>::value &&
    !std::is_same<detail::decay_t<U>, in_place_t>::value &&
    !std::is_same<expected<T, E>, detail::decay_t<U>>::value &&
    !std::is_same<unexpected<E>, detail::decay_t<U>>::value>;

template <class T, class E, class U, class G, class UR, class GR>
// SFINAE：允许从另一个 expected<U,G> 转换构造的条件：
// T 可由 UR、E 可由 GR 构造，且 expected<U,G> 不能直接构造/转换成 T(避免与拷贝构造冲突)。
using expected_enable_from_other = detail::enable_if_t<
    std::is_constructible<T, UR>::value &&
    std::is_constructible<E, GR>::value &&
    !std::is_constructible<T, expected<U, G> &>::value &&
    !std::is_constructible<T, expected<U, G> &&>::value &&
    !std::is_constructible<T, const expected<U, G> &>::value &&
    !std::is_constructible<T, const expected<U, G> &&>::value &&
    !std::is_convertible<expected<U, G> &, T>::value &&
    !std::is_convertible<expected<U, G> &&, T>::value &&
    !std::is_convertible<const expected<U, G> &, T>::value &&
    !std::is_convertible<const expected<U, G> &&, T>::value>;

template <class T, class U>
// 小工具：T 为 void 时直接给 true_type，否则用 U(用于让 void 版本跳过对 T 的特性要求)
using is_void_or = conditional_t<std::is_void<T>::value, std::true_type, U>;

template <class T>
using is_copy_constructible_or_void =  // 可拷贝构造，或 T=void
    is_void_or<T, std::is_copy_constructible<T>>;

template <class T>
using is_move_constructible_or_void =  // 可移动构造，或 T=void
    is_void_or<T, std::is_move_constructible<T>>;

template <class T>
using is_copy_assignable_or_void = is_void_or<T, std::is_copy_assignable<T>>;  // 可拷贝赋值，或 T=void

template <class T>
using is_move_assignable_or_void = is_void_or<T, std::is_move_assignable<T>>;  // 可移动赋值，或 T=void

} // namespace detail
namespace detail {
/*===========================================================================
 * 第三部分：存储层 expected_storage_base
 * 用联合体存放 T 或 unexpected<E>(外加 1 字节占位)，m_has_val 标记当前存的是哪一个；
 * 按 T/E 是否平凡析构组合出 6 个特化，尽量让 expected 的析构也保持平凡。
 *===========================================================================*/
struct no_init_t {};  // no_init_t：标签，表示“联合体先不构造任何成员”
static constexpr no_init_t no_init{};  // 其全局实例

// Implements the storage of the values, and ensures that the destructor is
// trivial if it can be.
//
// This specialization is for where neither `T` or `E` is trivially
// destructible, so the destructors must be called on destruction of the
// `expected`
template <class T, class E, bool = std::is_trivially_destructible<T>::value,
          bool = std::is_trivially_destructible<E>::value>
/*--- 特化①(主模板)：T、E 都不是平凡析构 -> 必须在析构里手动调用对应析构函数 ---*/
struct expected_storage_base {
  constexpr expected_storage_base() : m_val(T{}), m_has_val(true) {}  // 默认构造：就地值初始化一个 T，标记为“有值”
  constexpr expected_storage_base(no_init_t) : m_no_init(), m_has_val(false) {}  // no_init 构造：只激活占位字节，标记“无值”

  template <class... Args,
            detail::enable_if_t<std::is_constructible<T, Args &&...>::value> * =
                nullptr>
  constexpr expected_storage_base(in_place_t, Args &&...args)  // in_place：用 Args... 原地构造成功值 T
      : m_val(std::forward<Args>(args)...), m_has_val(true) {}

  template <class U, class... Args,
            detail::enable_if_t<std::is_constructible<
                T, std::initializer_list<U> &, Args &&...>::value> * = nullptr>
  constexpr expected_storage_base(in_place_t, std::initializer_list<U> il,  // in_place + 初始化列表构造 T
                                  Args &&...args)
      : m_val(il, std::forward<Args>(args)...), m_has_val(true) {}
  template <class... Args,
            detail::enable_if_t<std::is_constructible<E, Args &&...>::value> * =
                nullptr>
  constexpr explicit expected_storage_base(unexpect_t, Args &&...args)  // unexpect：用 Args... 原地构造错误 unexpected<E>
      : m_unexpect(std::forward<Args>(args)...), m_has_val(false) {}

  template <class U, class... Args,
            detail::enable_if_t<std::is_constructible<
                E, std::initializer_list<U> &, Args &&...>::value> * = nullptr>
  constexpr explicit expected_storage_base(unexpect_t,  // unexpect + 初始化列表构造错误(各特化结构相同)
                                           std::initializer_list<U> il,
                                           Args &&...args)
      : m_unexpect(il, std::forward<Args>(args)...), m_has_val(false) {}

  ~expected_storage_base() {  // 析构：按当前状态手动销毁 T 或 unexpected<E>
    if (m_has_val) {
      m_val.~T();
    } else {
      m_unexpect.~unexpected<E>();
    }
  }
  // 联合体：同一时刻只构造三者之一(T / 错误 / 占位字节)
  union {
    T m_val;  // 成功值
    unexpected<E> m_unexpect;  // 错误值
    char m_no_init;  // 未初始化时的 1 字节占位
  };
  bool m_has_val;  // true=当前存的是 T；false=当前存的是 unexpected<E>
};

// This specialization is for when both `T` and `E` are trivially-destructible,
// so the destructor of the `expected` can be trivial.
template <class T, class E>
/*--- 特化②：T、E 都平凡析构 -> 析构可 = default(平凡)，特殊成员全部默认 ---*/
struct expected_storage_base<T, E, true, true> {
  constexpr expected_storage_base() : m_val(T{}), m_has_val(true) {}
  constexpr expected_storage_base(no_init_t) : m_no_init(), m_has_val(false) {}

  template <class... Args,
            detail::enable_if_t<std::is_constructible<T, Args &&...>::value> * =
                nullptr>
  constexpr expected_storage_base(in_place_t, Args &&...args)
      : m_val(std::forward<Args>(args)...), m_has_val(true) {}

  template <class U, class... Args,
            detail::enable_if_t<std::is_constructible<
                T, std::initializer_list<U> &, Args &&...>::value> * = nullptr>
  constexpr expected_storage_base(in_place_t, std::initializer_list<U> il,
                                  Args &&...args)
      : m_val(il, std::forward<Args>(args)...), m_has_val(true) {}
  template <class... Args,
            detail::enable_if_t<std::is_constructible<E, Args &&...>::value> * =
                nullptr>
  constexpr explicit expected_storage_base(unexpect_t, Args &&...args)
      : m_unexpect(std::forward<Args>(args)...), m_has_val(false) {}

  template <class U, class... Args,
            detail::enable_if_t<std::is_constructible<
                E, std::initializer_list<U> &, Args &&...>::value> * = nullptr>
  constexpr explicit expected_storage_base(unexpect_t,
                                           std::initializer_list<U> il,
                                           Args &&...args)
      : m_unexpect(il, std::forward<Args>(args)...), m_has_val(false) {}

  expected_storage_base(const expected_storage_base &) = default;
  expected_storage_base(expected_storage_base &&) = default;
  expected_storage_base &operator=(const expected_storage_base &) = default;
  expected_storage_base &operator=(expected_storage_base &&) = default;
  ~expected_storage_base() = default;  // 双方都平凡析构，析构直接默认即可
  union {
    T m_val;
    unexpected<E> m_unexpect;
    char m_no_init;
  };
  bool m_has_val;
};

// T is trivial, E is not.
template <class T, class E>
/*--- 特化③：T 平凡析构、E 不平凡 -> 析构时只需在“错误态”销毁 unexpected<E> ---*/
struct expected_storage_base<T, E, true, false> {
  constexpr expected_storage_base() : m_val(T{}), m_has_val(true) {}
  TL_EXPECTED_MSVC2015_CONSTEXPR expected_storage_base(no_init_t)  // MSVC2015 兼容：此处不能加 constexpr
      : m_no_init(), m_has_val(false) {}

  template <class... Args,
            detail::enable_if_t<std::is_constructible<T, Args &&...>::value> * =
                nullptr>
  constexpr expected_storage_base(in_place_t, Args &&...args)
      : m_val(std::forward<Args>(args)...), m_has_val(true) {}

  template <class U, class... Args,
            detail::enable_if_t<std::is_constructible<
                T, std::initializer_list<U> &, Args &&...>::value> * = nullptr>
  constexpr expected_storage_base(in_place_t, std::initializer_list<U> il,
                                  Args &&...args)
      : m_val(il, std::forward<Args>(args)...), m_has_val(true) {}
  template <class... Args,
            detail::enable_if_t<std::is_constructible<E, Args &&...>::value> * =
                nullptr>
  constexpr explicit expected_storage_base(unexpect_t, Args &&...args)
      : m_unexpect(std::forward<Args>(args)...), m_has_val(false) {}

  template <class U, class... Args,
            detail::enable_if_t<std::is_constructible<
                E, std::initializer_list<U> &, Args &&...>::value> * = nullptr>
  constexpr explicit expected_storage_base(unexpect_t,
                                           std::initializer_list<U> il,
                                           Args &&...args)
      : m_unexpect(il, std::forward<Args>(args)...), m_has_val(false) {}

  expected_storage_base(const expected_storage_base &) = default;
  expected_storage_base(expected_storage_base &&) = default;
  expected_storage_base &operator=(const expected_storage_base &) = default;
  expected_storage_base &operator=(expected_storage_base &&) = default;
  ~expected_storage_base() {  // 仅错误态需要销毁 E(T 平凡析构，无需处理)
    if (!m_has_val) {
      m_unexpect.~unexpected<E>();
    }
  }

  union {
    T m_val;
    unexpected<E> m_unexpect;
    char m_no_init;
  };
  bool m_has_val;
};

// E is trivial, T is not.
template <class T, class E>
/*--- 特化④：T 不平凡析构、E 平凡 -> 析构时只需在“有值态”销毁 T ---*/
struct expected_storage_base<T, E, false, true> {
  constexpr expected_storage_base() : m_val(T{}), m_has_val(true) {}
  constexpr expected_storage_base(no_init_t) : m_no_init(), m_has_val(false) {}

  template <class... Args,
            detail::enable_if_t<std::is_constructible<T, Args &&...>::value> * =
                nullptr>
  constexpr expected_storage_base(in_place_t, Args &&...args)
      : m_val(std::forward<Args>(args)...), m_has_val(true) {}

  template <class U, class... Args,
            detail::enable_if_t<std::is_constructible<
                T, std::initializer_list<U> &, Args &&...>::value> * = nullptr>
  constexpr expected_storage_base(in_place_t, std::initializer_list<U> il,
                                  Args &&...args)
      : m_val(il, std::forward<Args>(args)...), m_has_val(true) {}
  template <class... Args,
            detail::enable_if_t<std::is_constructible<E, Args &&...>::value> * =
                nullptr>
  constexpr explicit expected_storage_base(unexpect_t, Args &&...args)
      : m_unexpect(std::forward<Args>(args)...), m_has_val(false) {}

  template <class U, class... Args,
            detail::enable_if_t<std::is_constructible<
                E, std::initializer_list<U> &, Args &&...>::value> * = nullptr>
  constexpr explicit expected_storage_base(unexpect_t,
                                           std::initializer_list<U> il,
                                           Args &&...args)
      : m_unexpect(il, std::forward<Args>(args)...), m_has_val(false) {}

  expected_storage_base(const expected_storage_base &) = default;
  expected_storage_base(expected_storage_base &&) = default;
  expected_storage_base &operator=(const expected_storage_base &) = default;
  expected_storage_base &operator=(expected_storage_base &&) = default;
  ~expected_storage_base() {  // 仅有值态需要销毁 T(E 平凡析构)
    if (m_has_val) {
      m_val.~T();
    }
  }
  union {
    T m_val;
    unexpected<E> m_unexpect;
    char m_no_init;
  };
  bool m_has_val;
};

// `T` is `void`, `E` is trivially-destructible
template <class E>
/*--- 特化⑤：T=void 且 E 平凡析构。void 没有真正的值，用 dummy 空类型占位 ---*/
struct expected_storage_base<void, E, false, true> {
#if __GNUC__ <= 5  // GCC4/5 在此处加 constexpr 有 bug，故该版本留空
  //no constexpr for GCC 4/5 bug
#else
  TL_EXPECTED_MSVC2015_CONSTEXPR
#endif
  expected_storage_base() : m_has_val(true) {}  // void 版默认构造：没有 T，只把状态置为有值
  constexpr expected_storage_base(no_init_t) : m_val(), m_has_val(false) {}  // void 版 no_init：激活 dummy 占位，状态无值

  constexpr expected_storage_base(in_place_t) : m_has_val(true) {}  // void 版 in_place：无需构造任何东西，仅标记有值

  template <class... Args,
            detail::enable_if_t<std::is_constructible<E, Args &&...>::value> * =
                nullptr>
  constexpr explicit expected_storage_base(unexpect_t, Args &&...args)
      : m_unexpect(std::forward<Args>(args)...), m_has_val(false) {}

  template <class U, class... Args,
            detail::enable_if_t<std::is_constructible<
                E, std::initializer_list<U> &, Args &&...>::value> * = nullptr>
  constexpr explicit expected_storage_base(unexpect_t,
                                           std::initializer_list<U> il,
                                           Args &&...args)
      : m_unexpect(il, std::forward<Args>(args)...), m_has_val(false) {}

  expected_storage_base(const expected_storage_base &) = default;
  expected_storage_base(expected_storage_base &&) = default;
  expected_storage_base &operator=(const expected_storage_base &) = default;
  expected_storage_base &operator=(expected_storage_base &&) = default;
  ~expected_storage_base() = default;
  struct dummy {};  // void 成功态的占位空类型
  union {
    unexpected<E> m_unexpect;
    dummy m_val;
  };
  bool m_has_val;
};

// `T` is `void`, `E` is not trivially-destructible
template <class E>
/*--- 特化⑥：T=void 且 E 不平凡析构 -> 错误态需手动销毁 unexpected<E> ---*/
struct expected_storage_base<void, E, false, false> {
  constexpr expected_storage_base() : m_dummy(), m_has_val(true) {}  // 默认构造占位字节并标记有值
  constexpr expected_storage_base(no_init_t) : m_dummy(), m_has_val(false) {}

  constexpr expected_storage_base(in_place_t) : m_dummy(), m_has_val(true) {}

  template <class... Args,
            detail::enable_if_t<std::is_constructible<E, Args &&...>::value> * =
                nullptr>
  constexpr explicit expected_storage_base(unexpect_t, Args &&...args)
      : m_unexpect(std::forward<Args>(args)...), m_has_val(false) {}

  template <class U, class... Args,
            detail::enable_if_t<std::is_constructible<
                E, std::initializer_list<U> &, Args &&...>::value> * = nullptr>
  constexpr explicit expected_storage_base(unexpect_t,
                                           std::initializer_list<U> il,
                                           Args &&...args)
      : m_unexpect(il, std::forward<Args>(args)...), m_has_val(false) {}

  expected_storage_base(const expected_storage_base &) = default;
  expected_storage_base(expected_storage_base &&) = default;
  expected_storage_base &operator=(const expected_storage_base &) = default;
  expected_storage_base &operator=(expected_storage_base &&) = default;
  ~expected_storage_base() {  // 仅错误态销毁 E
    if (!m_has_val) {
      m_unexpect.~unexpected<E>();
    }
  }

  union {
    unexpected<E> m_unexpect;
    char m_dummy;
  };
  bool m_has_val;
};
// This base class provides some handy member functions which can be used in
// further derived classes
template <class T, class E>
/*===========================================================================
 * 第四部分：操作层 expected_operations_base(泛型 T 版本)
 * 在存储层之上提供 placement-new 构造/销毁、强异常安全的 assign、各类 get/geterr。
 *===========================================================================*/
struct expected_operations_base : expected_storage_base<T, E> {  // 继承存储层，并复用其全部构造函数
  using expected_storage_base<T, E>::expected_storage_base;

  template <class... Args>
  // 在 m_val 存储位置上 placement-new 构造 T(此时该处尚未构造 T)
  void construct(Args &&...args) noexcept {  // 原地构造 T 并置“有值”
    new (std::addressof(this->m_val)) T(std::forward<Args>(args)...);
    this->m_has_val = true;
  }

  // void 版 assign：只在 有值<->无值 之间切换状态并构造/销毁 E
  template <class Rhs>
  void construct_with(Rhs &&rhs) noexcept {  // 从另一个同类的 get() 拷贝构造 T
    new (std::addressof(this->m_val)) T(std::forward<Rhs>(rhs).get());
    this->m_has_val = true;
  }

  template <class... Args>
  // 在 m_unexpect 位置 placement-new 构造 unexpected<E>
  void construct_error(Args &&...args) noexcept {  // 原地构造错误并置“无值”
    new (std::addressof(this->m_unexpect))
        unexpected<E>(std::forward<Args>(args)...);
    this->m_has_val = false;
  }

// assign 系列：在保持“强异常保证”(失败时 *this 状态不变)的前提下选最高效实现。
// 最棘手的情形：rhs 有值而 *this 当前是错误态(需要先销毁 E 再构造 T，构造 T 可能抛异常)。
#ifdef TL_EXPECTED_EXCEPTIONS_ENABLED

  // These assign overloads ensure that the most efficient assignment
  // implementation is used while maintaining the strong exception guarantee.
  // The problematic case is where rhs has a value, but *this does not.
  //
  // This overload handles the case where we can just copy-construct `T`
  // directly into place without throwing.
  template <class U = T,
            detail::enable_if_t<std::is_nothrow_copy_constructible<U>::value>
                * = nullptr>
  // 拷贝赋值情形 A：T 可 noexcept 拷贝构造 —— 直接原地拷贝，不会抛
  void assign(const expected_operations_base &rhs) noexcept {  // 当前错误、rhs有值：销毁 E 后直接构造 T；其余走公共逻辑
    if (!this->m_has_val && rhs.m_has_val) {
      geterr().~unexpected<E>();
      construct(rhs.get());
    } else {
      assign_common(rhs);
    }
  }

  // This overload handles the case where we can attempt to create a copy of
  // `T`, then no-throw move it into place if the copy was successful.
  template <class U = T,
            detail::enable_if_t<!std::is_nothrow_copy_constructible<U>::value &&
                                std::is_nothrow_move_constructible<U>::value>
                * = nullptr>
  // 拷贝赋值情形 B：T 拷贝会抛但移动不抛 —— 先在临时对象里拷贝，成功后再 no-throw 移入
  void assign(const expected_operations_base &rhs) noexcept {  // 先复制到 tmp，再销毁 E 并移动就位
    if (!this->m_has_val && rhs.m_has_val) {
      T tmp = rhs.get();
      geterr().~unexpected<E>();
      construct(std::move(tmp));
    } else {
      assign_common(rhs);
    }
  }

  // This overload is the worst-case, where we have to move-construct the
  // unexpected value into temporary storage, then try to copy the T into place.
  // If the construction succeeds, then everything is fine, but if it throws,
  // then we move the old unexpected value back into place before rethrowing the
  // exception.
  template <class U = T,
            detail::enable_if_t<!std::is_nothrow_copy_constructible<U>::value &&
                                !std::is_nothrow_move_constructible<U>::value>
                * = nullptr>
  // 拷贝赋值情形 C(最差)：T 拷贝、移动都可能抛 —— 先把旧 E 移到临时处保存，
  // 尝试构造 T，若抛异常则把 E 移回原位再 rethrow，保证强异常保证
  void assign(const expected_operations_base &rhs) {
    if (!this->m_has_val && rhs.m_has_val) {
      auto tmp = std::move(geterr());  // 先把旧错误值移到 tmp 保存
      geterr().~unexpected<E>();

#ifdef TL_EXPECTED_EXCEPTIONS_ENABLED
      try {
        construct(rhs.get());
      } catch (...) {
        geterr() = std::move(tmp);
        throw;
      }
#else
      construct(rhs.get());
#endif
    } else {
      assign_common(rhs);
    }
  }

  // These overloads do the same as above, but for rvalues
  template <class U = T,
            detail::enable_if_t<std::is_nothrow_move_constructible<U>::value>
                * = nullptr>
  // 移动赋值情形 A：T 可 noexcept 移动构造 —— 直接移动就位
  void assign(expected_operations_base &&rhs) noexcept {
    if (!this->m_has_val && rhs.m_has_val) {
      geterr().~unexpected<E>();
      construct(std::move(rhs).get());
    } else {
      assign_common(std::move(rhs));
    }
  }

  template <class U = T,
            detail::enable_if_t<!std::is_nothrow_move_constructible<U>::value>
                * = nullptr>
  // 移动赋值情形 B：T 移动可能抛 —— 同样先保存旧 E，失败则回滚
  void assign(expected_operations_base &&rhs) {
    if (!this->m_has_val && rhs.m_has_val) {
      auto tmp = std::move(geterr());
      geterr().~unexpected<E>();
#ifdef TL_EXPECTED_EXCEPTIONS_ENABLED
      try {
        construct(std::move(rhs).get());
      } catch (...) {
        geterr() = std::move(tmp);
        throw;
      }
#else
      construct(std::move(rhs).get());
#endif
    } else {
      assign_common(std::move(rhs));
    }
  }

#else

  // If exceptions are disabled then we can just copy-construct
  void assign(const expected_operations_base &rhs) noexcept {
    if (!this->m_has_val && rhs.m_has_val) {
      geterr().~unexpected<E>();
      construct(rhs.get());
    } else {
      assign_common(rhs);
    }
  }

  void assign(expected_operations_base &&rhs) noexcept {
    if (!this->m_has_val && rhs.m_has_val) {
      geterr().~unexpected<E>();
      construct(std::move(rhs).get());
    } else {
      assign_common(std::move(rhs));
    }
  }

#endif

  // The common part of move/copy assigning
  template <class Rhs>
  // 四种状态组合的公共赋值逻辑(有值/无值 × rhs 有值/无值)
  void assign_common(Rhs &&rhs) {  // *this 有值：rhs有值则直接赋值 T，否则销毁 T 改构造 E；*this 无值：仅当 rhs 也无值时赋值 E
    if (this->m_has_val) {
      if (rhs.m_has_val) {
        get() = std::forward<Rhs>(rhs).get();
      } else {
        destroy_val();
        construct_error(std::forward<Rhs>(rhs).geterr());
      }
    } else {
      if (!rhs.m_has_val) {
        geterr() = std::forward<Rhs>(rhs).geterr();
      }
    }
  }

  bool has_value() const { return this->m_has_val; }  // 是否持有成功值

  TL_EXPECTED_11_CONSTEXPR T &get() & { return this->m_val; }  // 左值取 T 引用
  constexpr const T &get() const & { return this->m_val; }  // const 左值取 const T&
  TL_EXPECTED_11_CONSTEXPR T &&get() && { return std::move(this->m_val); }  // 右值移动取 T
#ifndef TL_EXPECTED_NO_CONSTRR
  constexpr const T &&get() const && { return std::move(this->m_val); }  // const 右值移动取 T(老编译器用宏屏蔽)
#endif

  TL_EXPECTED_11_CONSTEXPR unexpected<E> &geterr() & {  // 左值取错误包装引用
    return this->m_unexpect;
  }
  constexpr const unexpected<E> &geterr() const & { return this->m_unexpect; }  // const 左值取 const 错误&
  TL_EXPECTED_11_CONSTEXPR unexpected<E> &&geterr() && {  // 右值移动取错误
    return std::move(this->m_unexpect);
  }
#ifndef TL_EXPECTED_NO_CONSTRR
  constexpr const unexpected<E> &&geterr() const && {  // const 右值移动取错误(老编译器屏蔽)
    return std::move(this->m_unexpect);
  }
#endif

  TL_EXPECTED_11_CONSTEXPR void destroy_val() { get().~T(); }  // 显式调用 T 的析构函数(销毁成功值)
};

// This base class provides some handy member functions which can be used in
// further derived classes
template <class E>
/*--- 操作层的 void 特化：成功态不携带值，construct 只翻转标志位，其余与泛型对称 ---*/
struct expected_operations_base<void, E> : expected_storage_base<void, E> {  // 继承 void 存储层
  using expected_storage_base<void, E>::expected_storage_base;

  template <class... Args>
  void construct() noexcept {  // void 版构造成功值：无 T 可构造，仅置有值标志
    this->m_has_val = true;
  }

  // This function doesn't use its argument, but needs it so that code in
  // levels above this can work independently of whether T is void
  template <class Rhs>
  void construct_with(Rhs &&) noexcept {  // 参数仅为与上层统一接口，void 下忽略之
    this->m_has_val = true;
  }

  template <class... Args>
  void construct_error(Args &&...args) noexcept {  // void 版：placement-new 构造错误并置无值
    new (std::addressof(this->m_unexpect))
        unexpected<E>(std::forward<Args>(args)...);
    this->m_has_val = false;
  }

  template <class Rhs>
  void assign(Rhs &&rhs) noexcept {
    if (!this->m_has_val) {
      if (rhs.m_has_val) {
        geterr().~unexpected<E>();
        construct();
      } else {
        geterr() = std::forward<Rhs>(rhs).geterr();
      }
    } else {
      if (!rhs.m_has_val) {
        construct_error(std::forward<Rhs>(rhs).geterr());
      }
    }
  }

  bool has_value() const { return this->m_has_val; }

  TL_EXPECTED_11_CONSTEXPR unexpected<E> &geterr() & {
    return this->m_unexpect;
  }
  constexpr const unexpected<E> &geterr() const & { return this->m_unexpect; }
  TL_EXPECTED_11_CONSTEXPR unexpected<E> &&geterr() && {
    return std::move(this->m_unexpect);
  }
#ifndef TL_EXPECTED_NO_CONSTRR
  constexpr const unexpected<E> &&geterr() const && {
    return std::move(this->m_unexpect);
  }
#endif

  // void 版销毁成功值是空操作(根本没有 T)
  TL_EXPECTED_11_CONSTEXPR void destroy_val() {
    // no-op
  }
};
// This class manages conditionally having a trivial copy constructor
// This specialization is for when T and E are trivially copy constructible
template <class T, class E,
          bool = is_void_or<T,TL_EXPECTED_IS_TRIVIALLY_COPY_CONSTRUCTIBLE(T)>::
                   value &&TL_EXPECTED_IS_TRIVIALLY_COPY_CONSTRUCTIBLE(E)::value,
          bool = (is_copy_constructible_or_void<T>::value &&
                  std::is_copy_constructible<E>::value)>
/*===========================================================================
 * 第五部分：按 T/E 特性条件性地提供“平凡/自定义”拷贝、移动、赋值的基类链
 * 主模板(满足平凡条件)只是透传继承；false 偏特化才写非平凡实现。
 *===========================================================================*/
// 拷贝构造基类主模板：T、E 都平凡可拷贝构造时，直接透传(拷贝构造保持平凡)
struct expected_copy_base : expected_operations_base<T, E> {
  using expected_operations_base<T, E>::expected_operations_base;
};

// This specialization is for when T or E are non-trivially copy constructible
template <class T, class E>
// 偏特化：T 或 E 非平凡可拷贝 -> 自定义拷贝构造：先 no_init，再按状态 placement-new 对应成员
struct expected_copy_base<T, E, false, true> : expected_operations_base<T, E> {
  using expected_operations_base<T, E>::expected_operations_base;

  expected_copy_base() = default;
  expected_copy_base(const expected_copy_base &rhs)  // 拷贝：不初始化联合体(no_init)，rhs有值则拷值，否则拷错误
      : expected_operations_base<T, E>(no_init) {
    if (rhs.has_value()) {
      this->construct_with(rhs);
    } else {
      this->construct_error(rhs.geterr());
    }
  }

  expected_copy_base(expected_copy_base &&rhs) = default;
  expected_copy_base &operator=(const expected_copy_base &rhs) = default;
  expected_copy_base &operator=(expected_copy_base &&rhs) = default;
};

// This class manages conditionally having a trivial move constructor
// Unfortunately there's no way to achieve this in GCC < 5 AFAIK, since it
// doesn't implement an analogue to std::is_trivially_move_constructible. We
// have to make do with a non-trivial move constructor even if T is trivially
// move constructible
#ifndef TL_EXPECTED_GCC49
template <class T, class E,
          bool = is_void_or<T, std::is_trivially_move_constructible<T>>::value
              &&std::is_trivially_move_constructible<E>::value>
// 移动构造基类主模板：T、E 都平凡可移动时透传(GCC4.9 无法判断，统一走非平凡分支)
struct expected_move_base : expected_copy_base<T, E> {
  using expected_copy_base<T, E>::expected_copy_base;
};
#else
template <class T, class E, bool = false>
struct expected_move_base;  // GCC4.9 下先声明主模板、强制走 false 偏特化
#endif

template <class T, class E>
// 移动构造偏特化：no_init 后按状态移动构造值或错误，noexcept 取决于 T 的移动构造
struct expected_move_base<T, E, false> : expected_copy_base<T, E> {
  using expected_copy_base<T, E>::expected_copy_base;

  expected_move_base() = default;
  expected_move_base(const expected_move_base &rhs) = default;

  expected_move_base(expected_move_base &&rhs) noexcept(  // 移动构造；noexcept 由 T 是否 noexcept 可移动构造决定
      std::is_nothrow_move_constructible<T>::value)
      : expected_copy_base<T, E>(no_init) {
    if (rhs.has_value()) {
      this->construct_with(std::move(rhs));
    } else {
      this->construct_error(std::move(rhs.geterr()));
    }
  }
  expected_move_base &operator=(const expected_move_base &rhs) = default;
  expected_move_base &operator=(expected_move_base &&rhs) = default;
};

// This class manages conditionally having a trivial copy assignment operator
template <class T, class E,
          bool = is_void_or<
              T, conjunction<TL_EXPECTED_IS_TRIVIALLY_COPY_ASSIGNABLE(T),
                             TL_EXPECTED_IS_TRIVIALLY_COPY_CONSTRUCTIBLE(T),
                             TL_EXPECTED_IS_TRIVIALLY_DESTRUCTIBLE(T)>>::value
              &&TL_EXPECTED_IS_TRIVIALLY_COPY_ASSIGNABLE(E)::value
                  &&TL_EXPECTED_IS_TRIVIALLY_COPY_CONSTRUCTIBLE(E)::value
                      &&TL_EXPECTED_IS_TRIVIALLY_DESTRUCTIBLE(E)::value,
          bool = (is_copy_constructible_or_void<T>::value &&
             std::is_copy_constructible<E>::value &&
             is_copy_assignable_or_void<T>::value &&
             std::is_copy_assignable<E>::value)>
// 拷贝赋值基类主模板：当 拷贝赋值/拷贝构造/析构 三者都平凡时透传，从而得到平凡拷贝赋值
struct expected_copy_assign_base : expected_move_base<T, E> {
  using expected_move_base<T, E>::expected_move_base;
};

template <class T, class E>
// 偏特化：非平凡时自定义拷贝赋值，内部调用前面的 assign() 强异常安全逻辑
struct expected_copy_assign_base<T, E, false, true>
    : expected_move_base<T, E> {
  using expected_move_base<T, E>::expected_move_base;

  expected_copy_assign_base() = default;
  expected_copy_assign_base(const expected_copy_assign_base &rhs) = default;

  expected_copy_assign_base(expected_copy_assign_base &&rhs) = default;
  expected_copy_assign_base &operator=(const expected_copy_assign_base &rhs) {  // 拷贝赋值委托给 assign()
    this->assign(rhs);
    return *this;
  }
  expected_copy_assign_base &
  operator=(expected_copy_assign_base &&rhs) = default;
};

// This class manages conditionally having a trivial move assignment operator
// Unfortunately there's no way to achieve this in GCC < 5 AFAIK, since it
// doesn't implement an analogue to std::is_trivially_move_assignable. We have
// to make do with a non-trivial move assignment operator even if T is trivially
// move assignable
#ifndef TL_EXPECTED_GCC49
template <class T, class E,
          bool =
              is_void_or<T, conjunction<std::is_trivially_destructible<T>,
                                        std::is_trivially_move_constructible<T>,
                                        std::is_trivially_move_assignable<T>>>::
                  value &&std::is_trivially_destructible<E>::value
                      &&std::is_trivially_move_constructible<E>::value
                          &&std::is_trivially_move_assignable<E>::value>
// 移动赋值基类主模板：移动构造/移动赋值/析构全平凡时透传(GCC4.9 同样强制走非平凡)
struct expected_move_assign_base : expected_copy_assign_base<T, E> {
  using expected_copy_assign_base<T, E>::expected_copy_assign_base;
};
#else
template <class T, class E, bool = false>
struct expected_move_assign_base;
#endif

template <class T, class E>
// 偏特化：非平凡移动赋值，委托给 assign(std::move(rhs))
struct expected_move_assign_base<T, E, false>
    : expected_copy_assign_base<T, E> {
  using expected_copy_assign_base<T, E>::expected_copy_assign_base;

  expected_move_assign_base() = default;
  expected_move_assign_base(const expected_move_assign_base &rhs) = default;

  expected_move_assign_base(expected_move_assign_base &&rhs) = default;

  expected_move_assign_base &
  operator=(const expected_move_assign_base &rhs) = default;

  expected_move_assign_base &
  operator=(expected_move_assign_base &&rhs) noexcept(  // 移动赋值；noexcept 取决于 T 的移动构造与移动赋值
      std::is_nothrow_move_constructible<T>::value
          &&std::is_nothrow_move_assignable<T>::value) {
    this->assign(std::move(rhs));
    return *this;
  }
};

// expected_delete_ctor_base will conditionally delete copy and move
// constructors depending on whether T is copy/move constructible
template <class T, class E,
          bool EnableCopy = (is_copy_constructible_or_void<T>::value &&
                             std::is_copy_constructible<E>::value),
          bool EnableMove = (is_move_constructible_or_void<T>::value &&
                             std::is_move_constructible<E>::value)>
/*--- 删除控制基类：依据 T/E 是否可拷贝/移动构造，用 4 个组合特化 delete 掉对应构造函数 ---*/
// 主模板(可拷贝且可移动)：全部默认
struct expected_delete_ctor_base {
  expected_delete_ctor_base() = default;
  expected_delete_ctor_base(const expected_delete_ctor_base &) = default;
  expected_delete_ctor_base(expected_delete_ctor_base &&) noexcept = default;
  expected_delete_ctor_base &
  operator=(const expected_delete_ctor_base &) = default;
  expected_delete_ctor_base &
  operator=(expected_delete_ctor_base &&) noexcept = default;
};

template <class T, class E>
struct expected_delete_ctor_base<T, E, true, false> {  // 可拷贝、不可移动 => delete 移动构造
  expected_delete_ctor_base() = default;
  expected_delete_ctor_base(const expected_delete_ctor_base &) = default;
  expected_delete_ctor_base(expected_delete_ctor_base &&) noexcept = delete;  // 删除移动构造
  expected_delete_ctor_base &
  operator=(const expected_delete_ctor_base &) = default;
  expected_delete_ctor_base &
  operator=(expected_delete_ctor_base &&) noexcept = default;
};

template <class T, class E>
struct expected_delete_ctor_base<T, E, false, true> {  // 不可拷贝、可移动 => delete 拷贝构造
  expected_delete_ctor_base() = default;
  expected_delete_ctor_base(const expected_delete_ctor_base &) = delete;  // 删除拷贝构造
  expected_delete_ctor_base(expected_delete_ctor_base &&) noexcept = default;
  expected_delete_ctor_base &
  operator=(const expected_delete_ctor_base &) = default;
  expected_delete_ctor_base &
  operator=(expected_delete_ctor_base &&) noexcept = default;
};

template <class T, class E>
struct expected_delete_ctor_base<T, E, false, false> {  // 既不可拷贝也不可移动 => 两个构造都 delete
  expected_delete_ctor_base() = default;
  expected_delete_ctor_base(const expected_delete_ctor_base &) = delete;
  expected_delete_ctor_base(expected_delete_ctor_base &&) noexcept = delete;
  expected_delete_ctor_base &
  operator=(const expected_delete_ctor_base &) = default;
  expected_delete_ctor_base &
  operator=(expected_delete_ctor_base &&) noexcept = default;
};

// expected_delete_assign_base will conditionally delete copy and move
// constructors depending on whether T and E are copy/move constructible +
// assignable
template <class T, class E,
          bool EnableCopy = (is_copy_constructible_or_void<T>::value &&
                             std::is_copy_constructible<E>::value &&
                             is_copy_assignable_or_void<T>::value &&
                             std::is_copy_assignable<E>::value),
          bool EnableMove = (is_move_constructible_or_void<T>::value &&
                             std::is_move_constructible<E>::value &&
                             is_move_assignable_or_void<T>::value &&
                             std::is_move_assignable<E>::value)>
/*--- 同理，按“可构造且可赋值”组合 delete 掉拷贝/移动赋值运算符 ---*/
// 主模板：全部默认
struct expected_delete_assign_base {
  expected_delete_assign_base() = default;
  expected_delete_assign_base(const expected_delete_assign_base &) = default;
  expected_delete_assign_base(expected_delete_assign_base &&) noexcept =
      default;
  expected_delete_assign_base &
  operator=(const expected_delete_assign_base &) = default;
  expected_delete_assign_base &
  operator=(expected_delete_assign_base &&) noexcept = default;
};

template <class T, class E>
struct expected_delete_assign_base<T, E, true, false> {  // 可拷贝赋值、不可移动赋值 => delete 移动赋值
  expected_delete_assign_base() = default;
  expected_delete_assign_base(const expected_delete_assign_base &) = default;
  expected_delete_assign_base(expected_delete_assign_base &&) noexcept =
      default;
  expected_delete_assign_base &
  operator=(const expected_delete_assign_base &) = default;
  expected_delete_assign_base &
  operator=(expected_delete_assign_base &&) noexcept = delete;  // 删除移动赋值
};

template <class T, class E>
struct expected_delete_assign_base<T, E, false, true> {  // 不可拷贝赋值、可移动赋值 => delete 拷贝赋值
  expected_delete_assign_base() = default;
  expected_delete_assign_base(const expected_delete_assign_base &) = default;
  expected_delete_assign_base(expected_delete_assign_base &&) noexcept =
      default;
  expected_delete_assign_base &
  operator=(const expected_delete_assign_base &) = delete;  // 删除拷贝赋值
  expected_delete_assign_base &
  operator=(expected_delete_assign_base &&) noexcept = default;
};

template <class T, class E>
struct expected_delete_assign_base<T, E, false, false> {  // 两种赋值都不可 => 全部 delete
  expected_delete_assign_base() = default;
  expected_delete_assign_base(const expected_delete_assign_base &) = default;
  expected_delete_assign_base(expected_delete_assign_base &&) noexcept =
      default;
  expected_delete_assign_base &
  operator=(const expected_delete_assign_base &) = delete;
  expected_delete_assign_base &
  operator=(expected_delete_assign_base &&) noexcept = delete;
};

// This is needed to be able to construct the expected_default_ctor_base which
// follows, while still conditionally deleting the default constructor.
// 辅助标签：让派生类能够绕开“被删除的默认构造”，显式调用基类构造
struct default_constructor_tag {
  explicit constexpr default_constructor_tag() = default;
};

// expected_default_ctor_base will ensure that expected has a deleted default
// constructor if T is not default constructible.
// This specialization is for when T is default constructible
template <class T, class E,
          bool Enable =
              std::is_default_constructible<T>::value || std::is_void<T>::value>
/*--- 默认构造控制：T 可默认构造(或为 void)时默认构造可用 ---*/
struct expected_default_ctor_base {
  constexpr expected_default_ctor_base() noexcept = default;
  constexpr expected_default_ctor_base(
      expected_default_ctor_base const &) noexcept = default;
  constexpr expected_default_ctor_base(expected_default_ctor_base &&) noexcept =
      default;
  expected_default_ctor_base &
  operator=(expected_default_ctor_base const &) noexcept = default;
  expected_default_ctor_base &
  operator=(expected_default_ctor_base &&) noexcept = default;

  constexpr explicit expected_default_ctor_base(default_constructor_tag) {}  // 供派生类用标签显式调用的构造(主模板)
};

// This specialization is for when T is not default constructible
template <class T, class E>
// 偏特化：T 不可默认构造 => 删除 expected 的默认构造
struct expected_default_ctor_base<T, E, false> {  // 默认构造被 delete
  constexpr expected_default_ctor_base() noexcept = delete;
  constexpr expected_default_ctor_base(
      expected_default_ctor_base const &) noexcept = default;
  constexpr expected_default_ctor_base(expected_default_ctor_base &&) noexcept =
      default;
  expected_default_ctor_base &
  operator=(expected_default_ctor_base const &) noexcept = default;
  expected_default_ctor_base &
  operator=(expected_default_ctor_base &&) noexcept = default;

  constexpr explicit expected_default_ctor_base(default_constructor_tag) {}  // 同上(false 偏特化也保留这个标签构造)
};
} // namespace detail

template <class E>
/*--- 访问错误态的值(value())时抛出的异常，内部拷贝一份错误值 E ---*/
class bad_expected_access : public std::exception {
public:
  explicit bad_expected_access(E e) : m_val(std::move(e)) {}  // 构造时保存错误值

  virtual const char *what() const noexcept override {  // 异常说明文字
    return "Bad expected access";
  }

  const E &error() const & { return m_val; }  // 四种引用限定符取出错误值
  E &error() & { return m_val; }
  const E &&error() const && { return std::move(m_val); }
  E &&error() && { return std::move(m_val); }

private:
  E m_val;
};
/// An `expected<T, E>` object is an object that contains the storage for
/// another object and manages the lifetime of this contained object `T`.
/// Alternatively it could contain the storage for another unexpected object
/// `E`. The contained object may not be initialized after the expected object
/// has been initialized, and may not be destroyed before the expected object
/// has been destroyed. The initialization state of the contained object is
/// tracked by the expected object.
template <class T, class E>
/*===========================================================================
 * 第六部分：对外主类 expected<T,E>
 * 私有继承四个控制基类(存储/特殊成员/默认构造都由它们决定)，自身实现观察者与单子 API。
 *===========================================================================*/
class TL_EXPECTED_NODISCARD expected  // 结果不可忽略；私有继承实现继承(对外隐藏基类细节)
    : private detail::expected_move_assign_base<T, E>,
      private detail::expected_delete_ctor_base<T, E>,
      private detail::expected_delete_assign_base<T, E>,
      private detail::expected_default_ctor_base<T, E> {
  static_assert(!std::is_reference<T>::value, "T must not be a reference");  // 编译期约束：成功类型 T 不能是引用
  static_assert(!std::is_same<T, std::remove_cv<in_place_t>::type>::value,  // T 不能是 in_place_t 标签
                "T must not be in_place_t");
  static_assert(!std::is_same<T, std::remove_cv<unexpect_t>::type>::value,  // T 不能是 unexpect_t 标签
                "T must not be unexpect_t");
  static_assert(
      !std::is_same<T, typename std::remove_cv<unexpected<E>>::type>::value,  // T 不能就是 unexpected<E>(否则语义冲突)
      "T must not be unexpected<E>");
  static_assert(!std::is_reference<E>::value, "E must not be a reference");  // 错误类型 E 也不能是引用

  T *valptr() { return std::addressof(this->m_val); }  // 取联合体中 T 的原始地址(addressof 能绕过重载的 &)
  const T *valptr() const { return std::addressof(this->m_val); }  // const 版本
  unexpected<E> *errptr() { return std::addressof(this->m_unexpect); }  // 取联合体中错误对象的地址
  const unexpected<E> *errptr() const {
    return std::addressof(this->m_unexpect);
  }

  template <class U = T, detail::enable_if_t<!std::is_void<U>::value> * = nullptr>
  // 私有的 val()/err()：直接访问联合体成员(不做状态检查)，供类内部使用
  TL_EXPECTED_11_CONSTEXPR U &val() {
    return this->m_val;
  }
  TL_EXPECTED_11_CONSTEXPR unexpected<E> &err() { return this->m_unexpect; }  // 私有：取错误对象(不检查状态)

  template <class U = T, detail::enable_if_t<!std::is_void<U>::value> * = nullptr>
  constexpr const U &val() const {  // 私有 const 版取值
    return this->m_val;
  }
  constexpr const unexpected<E> &err() const { return this->m_unexpect; }  // 私有 const 版取错

  using impl_base = detail::expected_move_assign_base<T, E>;  // 实现基类别名(构造时转发)
  using ctor_base = detail::expected_default_ctor_base<T, E>;  // 默认构造控制基类别名

public:
  typedef T value_type;  // 对外暴露：成功值类型
  typedef E error_type;  // 对外暴露：错误值类型
  typedef unexpected<E> unexpected_type;  // 对外暴露：错误包装类型

#if defined(TL_EXPECTED_CXX14) && !defined(TL_EXPECTED_GCC49) &&               \
    !defined(TL_EXPECTED_GCC54) && !defined(TL_EXPECTED_GCC55)
  template <class F>
  /*---- 单子接口① and_then(bind/flatMap)：有值才调用 f(*this)，f 必须返回 expected；
      无值则原样把错误透传。按 &/&&/const&/const&& 给出四种重载；CXX14 与 C++11 各一套写法 ----*/
  TL_EXPECTED_11_CONSTEXPR auto and_then(F &&f) & {
    return and_then_impl(*this, std::forward<F>(f));
  }
  template <class F>
  TL_EXPECTED_11_CONSTEXPR auto and_then(F &&f) && {
    return and_then_impl(std::move(*this), std::forward<F>(f));
  }
  template <class F>
  constexpr auto and_then(F &&f) const & {
    return and_then_impl(*this, std::forward<F>(f));
  }

#ifndef TL_EXPECTED_NO_CONSTRR
  template <class F>
  constexpr auto and_then(F &&f) const && {
    return and_then_impl(std::move(*this), std::forward<F>(f));
  }
#endif
#else
  template <class F>
  TL_EXPECTED_11_CONSTEXPR auto
  and_then(F &&f) & -> decltype(and_then_impl(std::declval<expected &>(),
                                              std::forward<F>(f))) {
    return and_then_impl(*this, std::forward<F>(f));
  }
  template <class F>
  TL_EXPECTED_11_CONSTEXPR auto
  and_then(F &&f) && -> decltype(and_then_impl(std::declval<expected &&>(),
                                               std::forward<F>(f))) {
    return and_then_impl(std::move(*this), std::forward<F>(f));
  }
  template <class F>
  constexpr auto and_then(F &&f) const & -> decltype(and_then_impl(
      std::declval<expected const &>(), std::forward<F>(f))) {
    return and_then_impl(*this, std::forward<F>(f));
  }

#ifndef TL_EXPECTED_NO_CONSTRR
  template <class F>
  constexpr auto and_then(F &&f) const && -> decltype(and_then_impl(
      std::declval<expected const &&>(), std::forward<F>(f))) {
    return and_then_impl(std::move(*this), std::forward<F>(f));
  }
#endif
#endif

#if defined(TL_EXPECTED_CXX14) && !defined(TL_EXPECTED_GCC49) &&               \
    !defined(TL_EXPECTED_GCC54) && !defined(TL_EXPECTED_GCC55)
  template <class F>
  /*---- 单子接口② map：有值时对值调用 f 并把结果包成新 expected；无值透传错误。
      transform 是 map 的别名(下方实现完全相同)。----*/
  TL_EXPECTED_11_CONSTEXPR auto map(F &&f) & {
    return expected_map_impl(*this, std::forward<F>(f));
  }
  template <class F>
  TL_EXPECTED_11_CONSTEXPR auto map(F &&f) && {
    return expected_map_impl(std::move(*this), std::forward<F>(f));
  }
  template <class F>
  constexpr auto map(F &&f) const & {
    return expected_map_impl(*this, std::forward<F>(f));
  }
  template <class F>
  constexpr auto map(F &&f) const && {
    return expected_map_impl(std::move(*this), std::forward<F>(f));
  }
#else
  template <class F>
  TL_EXPECTED_11_CONSTEXPR decltype(expected_map_impl(
      std::declval<expected &>(), std::declval<F &&>()))
  map(F &&f) & {
    return expected_map_impl(*this, std::forward<F>(f));
  }
  template <class F>
  TL_EXPECTED_11_CONSTEXPR decltype(expected_map_impl(std::declval<expected>(),
                                                      std::declval<F &&>()))
  map(F &&f) && {
    return expected_map_impl(std::move(*this), std::forward<F>(f));
  }
  template <class F>
  constexpr decltype(expected_map_impl(std::declval<const expected &>(),
                                       std::declval<F &&>()))
  map(F &&f) const & {
    return expected_map_impl(*this, std::forward<F>(f));
  }

#ifndef TL_EXPECTED_NO_CONSTRR
  template <class F>
  constexpr decltype(expected_map_impl(std::declval<const expected &&>(),
                                       std::declval<F &&>()))
  map(F &&f) const && {
    return expected_map_impl(std::move(*this), std::forward<F>(f));
  }
#endif
#endif

#if defined(TL_EXPECTED_CXX14) && !defined(TL_EXPECTED_GCC49) &&               \
    !defined(TL_EXPECTED_GCC54) && !defined(TL_EXPECTED_GCC55)
  template <class F>
  /*---- transform：与 map 等价的别名 ----*/
  TL_EXPECTED_11_CONSTEXPR auto transform(F &&f) & {
    return expected_map_impl(*this, std::forward<F>(f));
  }
  template <class F>
  TL_EXPECTED_11_CONSTEXPR auto transform(F &&f) && {
    return expected_map_impl(std::move(*this), std::forward<F>(f));
  }
  template <class F>
  constexpr auto transform(F &&f) const & {
    return expected_map_impl(*this, std::forward<F>(f));
  }
  template <class F>
  constexpr auto transform(F &&f) const && {
    return expected_map_impl(std::move(*this), std::forward<F>(f));
  }
#else
  template <class F>
  TL_EXPECTED_11_CONSTEXPR decltype(expected_map_impl(
      std::declval<expected &>(), std::declval<F &&>()))
  transform(F &&f) & {
    return expected_map_impl(*this, std::forward<F>(f));
  }
  template <class F>
  TL_EXPECTED_11_CONSTEXPR decltype(expected_map_impl(std::declval<expected>(),
                                                      std::declval<F &&>()))
  transform(F &&f) && {
    return expected_map_impl(std::move(*this), std::forward<F>(f));
  }
  template <class F>
  constexpr decltype(expected_map_impl(std::declval<const expected &>(),
                                       std::declval<F &&>()))
  transform(F &&f) const & {
    return expected_map_impl(*this, std::forward<F>(f));
  }

#ifndef TL_EXPECTED_NO_CONSTRR
  template <class F>
  constexpr decltype(expected_map_impl(std::declval<const expected &&>(),
                                       std::declval<F &&>()))
  transform(F &&f) const && {
    return expected_map_impl(std::move(*this), std::forward<F>(f));
  }
#endif
#endif

#if defined(TL_EXPECTED_CXX14) && !defined(TL_EXPECTED_GCC49) &&               \
    !defined(TL_EXPECTED_GCC54) && !defined(TL_EXPECTED_GCC55)
  template <class F>
  /*---- 单子接口③ map_error：与 map 相反，只在“错误态”对错误调用 f 做变换；有值原样保留。
      transform_error 是其别名。----*/
  TL_EXPECTED_11_CONSTEXPR auto map_error(F &&f) & {
    return map_error_impl(*this, std::forward<F>(f));
  }
  template <class F>
  TL_EXPECTED_11_CONSTEXPR auto map_error(F &&f) && {
    return map_error_impl(std::move(*this), std::forward<F>(f));
  }
  template <class F>
  constexpr auto map_error(F &&f) const & {
    return map_error_impl(*this, std::forward<F>(f));
  }
  template <class F>
  constexpr auto map_error(F &&f) const && {
    return map_error_impl(std::move(*this), std::forward<F>(f));
  }
#else
  template <class F>
  TL_EXPECTED_11_CONSTEXPR decltype(map_error_impl(std::declval<expected &>(),
                                                   std::declval<F &&>()))
  map_error(F &&f) & {
    return map_error_impl(*this, std::forward<F>(f));
  }
  template <class F>
  TL_EXPECTED_11_CONSTEXPR decltype(map_error_impl(std::declval<expected &&>(),
                                                   std::declval<F &&>()))
  map_error(F &&f) && {
    return map_error_impl(std::move(*this), std::forward<F>(f));
  }
  template <class F>
  constexpr decltype(map_error_impl(std::declval<const expected &>(),
                                    std::declval<F &&>()))
  map_error(F &&f) const & {
    return map_error_impl(*this, std::forward<F>(f));
  }

#ifndef TL_EXPECTED_NO_CONSTRR
  template <class F>
  constexpr decltype(map_error_impl(std::declval<const expected &&>(),
                                    std::declval<F &&>()))
  map_error(F &&f) const && {
    return map_error_impl(std::move(*this), std::forward<F>(f));
  }
#endif
#endif
#if defined(TL_EXPECTED_CXX14) && !defined(TL_EXPECTED_GCC49) &&               \
    !defined(TL_EXPECTED_GCC54) && !defined(TL_EXPECTED_GCC55)
  template <class F>
  /*---- transform_error：与 map_error 等价的别名 ----*/
  TL_EXPECTED_11_CONSTEXPR auto transform_error(F &&f) & {
    return map_error_impl(*this, std::forward<F>(f));
  }
  template <class F>
  TL_EXPECTED_11_CONSTEXPR auto transform_error(F &&f) && {
    return map_error_impl(std::move(*this), std::forward<F>(f));
  }
  template <class F>
  constexpr auto transform_error(F &&f) const & {
    return map_error_impl(*this, std::forward<F>(f));
  }
  template <class F>
  constexpr auto transform_error(F &&f) const && {
    return map_error_impl(std::move(*this), std::forward<F>(f));
  }
#else
  template <class F>
  TL_EXPECTED_11_CONSTEXPR decltype(map_error_impl(std::declval<expected &>(),
                                                   std::declval<F &&>()))
  transform_error(F &&f) & {
    return map_error_impl(*this, std::forward<F>(f));
  }
  template <class F>
  TL_EXPECTED_11_CONSTEXPR decltype(map_error_impl(std::declval<expected &&>(),
                                                   std::declval<F &&>()))
  transform_error(F &&f) && {
    return map_error_impl(std::move(*this), std::forward<F>(f));
  }
  template <class F>
  constexpr decltype(map_error_impl(std::declval<const expected &>(),
                                    std::declval<F &&>()))
  transform_error(F &&f) const & {
    return map_error_impl(*this, std::forward<F>(f));
  }

#ifndef TL_EXPECTED_NO_CONSTRR
  template <class F>
  constexpr decltype(map_error_impl(std::declval<const expected &&>(),
                                    std::declval<F &&>()))
  transform_error(F &&f) const && {
    return map_error_impl(std::move(*this), std::forward<F>(f));
  }
#endif
#endif

  template <class F>
  /*---- 单子接口④ or_else：错误态才调用 f(错误)用于“兜底/恢复”；有值则原样返回自身 ----*/
  expected TL_EXPECTED_11_CONSTEXPR or_else(F &&f) & {
    return or_else_impl(*this, std::forward<F>(f));
  }

  template <class F>
  expected TL_EXPECTED_11_CONSTEXPR or_else(F &&f) && {
    return or_else_impl(std::move(*this), std::forward<F>(f));
  }

  template <class F>
  expected constexpr or_else(F &&f) const & {
    return or_else_impl(*this, std::forward<F>(f));
  }

#ifndef TL_EXPECTED_NO_CONSTRR
  template <class F>
  expected constexpr or_else(F &&f) const && {
    return or_else_impl(std::move(*this), std::forward<F>(f));
  }
#endif
  /*===========================================================================
 * 第七部分：构造函数(默认/拷贝移动/in_place/unexpected/unexpect/跨类型转换/万能值)
 *===========================================================================*/
  constexpr expected() = default;  // 默认构造(是否被删除由 default_ctor_base 决定)
  constexpr expected(const expected &rhs) = default;  // 拷贝构造默认
  constexpr expected(expected &&rhs) = default;  // 移动构造默认
  expected &operator=(const expected &rhs) = default;  // 拷贝赋值默认
  expected &operator=(expected &&rhs) = default;  // 移动赋值默认

  template <class... Args,
            detail::enable_if_t<std::is_constructible<T, Args &&...>::value> * =
                nullptr>
  constexpr expected(in_place_t, Args &&...args)  // in_place 原地构造成功值 T
      : impl_base(in_place, std::forward<Args>(args)...),
        ctor_base(detail::default_constructor_tag{}) {}

  template <class U, class... Args,
            detail::enable_if_t<std::is_constructible<
                T, std::initializer_list<U> &, Args &&...>::value> * = nullptr>
  constexpr expected(in_place_t, std::initializer_list<U> il, Args &&...args)  // in_place + 初始化列表构造 T
      : impl_base(in_place, il, std::forward<Args>(args)...),
        ctor_base(detail::default_constructor_tag{}) {}

  template <class G = E,
            detail::enable_if_t<std::is_constructible<E, const G &>::value> * =
                nullptr,
            detail::enable_if_t<!std::is_convertible<const G &, E>::value> * =
                nullptr>
  // 由 unexpected 构造错误态：分“E 不可隐式转换=>explicit”与“可隐式转换=>非 explicit”两套
  explicit constexpr expected(const unexpected<G> &e)  // const& 且不可隐式转换：显式
      : impl_base(unexpect, e.value()),
        ctor_base(detail::default_constructor_tag{}) {}

  template <
      class G = E,
      detail::enable_if_t<std::is_constructible<E, const G &>::value> * =
          nullptr,
      detail::enable_if_t<std::is_convertible<const G &, E>::value> * = nullptr>
  constexpr expected(unexpected<G> const &e)  // const& 且可隐式转换：隐式
      : impl_base(unexpect, e.value()),
        ctor_base(detail::default_constructor_tag{}) {}

  template <
      class G = E,
      detail::enable_if_t<std::is_constructible<E, G &&>::value> * = nullptr,
      detail::enable_if_t<!std::is_convertible<G &&, E>::value> * = nullptr>
  explicit constexpr expected(unexpected<G> &&e) noexcept(  // 右值且不可隐式转换：显式，noexcept 取决于 E 构造
      std::is_nothrow_constructible<E, G &&>::value)
      : impl_base(unexpect, std::move(e.value())),
        ctor_base(detail::default_constructor_tag{}) {}

  template <
      class G = E,
      detail::enable_if_t<std::is_constructible<E, G &&>::value> * = nullptr,
      detail::enable_if_t<std::is_convertible<G &&, E>::value> * = nullptr>
  constexpr expected(unexpected<G> &&e) noexcept(  // 右值且可隐式转换：隐式
      std::is_nothrow_constructible<E, G &&>::value)
      : impl_base(unexpect, std::move(e.value())),
        ctor_base(detail::default_constructor_tag{}) {}

  template <class... Args,
            detail::enable_if_t<std::is_constructible<E, Args &&...>::value> * =
                nullptr>
  constexpr explicit expected(unexpect_t, Args &&...args)  // unexpect 标签原地构造错误 E
      : impl_base(unexpect, std::forward<Args>(args)...),
        ctor_base(detail::default_constructor_tag{}) {}

  template <class U, class... Args,
            detail::enable_if_t<std::is_constructible<
                E, std::initializer_list<U> &, Args &&...>::value> * = nullptr>
  constexpr explicit expected(unexpect_t, std::initializer_list<U> il,  // unexpect + 初始化列表构造 E
                              Args &&...args)
      : impl_base(unexpect, il, std::forward<Args>(args)...),
        ctor_base(detail::default_constructor_tag{}) {}

  template <class U, class G,
            detail::enable_if_t<!(std::is_convertible<U const &, T>::value &&
                                 std::is_convertible<G const &, E>::value)> * =
                nullptr,
            detail::expected_enable_from_other<T, E, U, G, const U &, const G &>
                * = nullptr>
  // 跨 expected<U,G> 转换构造：同样按(U,G)能否隐式转换到(T,E)分 explicit/隐式两套，const&/&& 各一
  explicit TL_EXPECTED_11_CONSTEXPR expected(const expected<U, G> &rhs)  // const& 不可隐式转换：显式；按 rhs 状态构造值或错误
      : ctor_base(detail::default_constructor_tag{}) {
    if (rhs.has_value()) {
      this->construct(*rhs);
    } else {
      this->construct_error(rhs.error());
    }
  }

  template <class U, class G,
            detail::enable_if_t<(std::is_convertible<U const &, T>::value &&
                                 std::is_convertible<G const &, E>::value)> * =
                nullptr,
            detail::expected_enable_from_other<T, E, U, G, const U &, const G &>
                * = nullptr>
  TL_EXPECTED_11_CONSTEXPR expected(const expected<U, G> &rhs)  // const& 可隐式转换：隐式
      : ctor_base(detail::default_constructor_tag{}) {
    if (rhs.has_value()) {
      this->construct(*rhs);
    } else {
      this->construct_error(rhs.error());
    }
  }

  template <
      class U, class G,
      detail::enable_if_t<!(std::is_convertible<U &&, T>::value &&
                            std::is_convertible<G &&, E>::value)> * = nullptr,
      detail::expected_enable_from_other<T, E, U, G, U &&, G &&> * = nullptr>
  explicit TL_EXPECTED_11_CONSTEXPR expected(expected<U, G> &&rhs)  // && 不可隐式转换：显式移动版
      : ctor_base(detail::default_constructor_tag{}) {
    if (rhs.has_value()) {
      this->construct(std::move(*rhs));
    } else {
      this->construct_error(std::move(rhs.error()));
    }
  }

  template <
      class U, class G,
      detail::enable_if_t<(std::is_convertible<U &&, T>::value &&
                           std::is_convertible<G &&, E>::value)> * = nullptr,
      detail::expected_enable_from_other<T, E, U, G, U &&, G &&> * = nullptr>
  TL_EXPECTED_11_CONSTEXPR expected(expected<U, G> &&rhs)  // && 可隐式转换：隐式移动版
      : ctor_base(detail::default_constructor_tag{}) {
    if (rhs.has_value()) {
      this->construct(std::move(*rhs));
    } else {
      this->construct_error(std::move(rhs.error()));
    }
  }

  template <
      class U = T,
      detail::enable_if_t<!std::is_convertible<U &&, T>::value> * = nullptr,
      detail::expected_enable_forward_value<T, E, U> * = nullptr>
  // 万能值构造：把任意可构造 T 的 U 当作成功值(委托给 in_place 构造)；
  // 用 expected_enable_forward_value 排除 in_place/expected/unexpected，按可否隐式转换分 explicit。
  explicit TL_EXPECTED_MSVC2015_CONSTEXPR expected(U &&v)  // 不可隐式转换：显式
      : expected(in_place, std::forward<U>(v)) {}

  template <
      class U = T,
      detail::enable_if_t<std::is_convertible<U &&, T>::value> * = nullptr,
      detail::expected_enable_forward_value<T, E, U> * = nullptr>
  TL_EXPECTED_MSVC2015_CONSTEXPR expected(U &&v)  // 可隐式转换：隐式；均委托 in_place 构造
      : expected(in_place, std::forward<U>(v)) {}

  template <
      class U = T, class G = T,
      detail::enable_if_t<std::is_nothrow_constructible<T, U &&>::value> * =
          nullptr,
      detail::enable_if_t<!std::is_void<G>::value> * = nullptr,
      detail::enable_if_t<
          (!std::is_same<expected<T, E>, detail::decay_t<U>>::value &&
           !detail::conjunction<std::is_scalar<T>,
                                std::is_same<T, detail::decay_t<U>>>::value &&
           std::is_constructible<T, U>::value &&
           std::is_assignable<G &, U>::value &&
           std::is_nothrow_move_constructible<E>::value)> * = nullptr>
  /*===========================================================================
 * 第八部分：赋值运算符(万能值 / unexpected)与 emplace
 *===========================================================================*/
  // 万能值赋值 A：T 可 noexcept 构造 —— 有值直接赋值；错误态销毁 E 后直接 placement-new T
  expected &operator=(U &&v) {  // 错误态切换为有值态
    if (has_value()) {
      val() = std::forward<U>(v);
    } else {
      err().~unexpected<E>();
      ::new (valptr()) T(std::forward<U>(v));
      this->m_has_val = true;
    }

    return *this;
  }

  template <
      class U = T, class G = T,
      detail::enable_if_t<!std::is_nothrow_constructible<T, U &&>::value> * =
          nullptr,
      detail::enable_if_t<!std::is_void<U>::value> * = nullptr,
      detail::enable_if_t<
          (!std::is_same<expected<T, E>, detail::decay_t<U>>::value &&
           !detail::conjunction<std::is_scalar<T>,
                                std::is_same<T, detail::decay_t<U>>>::value &&
           std::is_constructible<T, U>::value &&
           std::is_assignable<G &, U>::value &&
           std::is_nothrow_move_constructible<E>::value)> * = nullptr>
  // 万能值赋值 B：T 构造可能抛 —— 先保存旧 E，构造失败回滚后 rethrow(强异常保证)
  expected &operator=(U &&v) {
    if (has_value()) {
      val() = std::forward<U>(v);
    } else {
      auto tmp = std::move(err());
      err().~unexpected<E>();

#ifdef TL_EXPECTED_EXCEPTIONS_ENABLED
      try {
        ::new (valptr()) T(std::forward<U>(v));
        this->m_has_val = true;
      } catch (...) {
        err() = std::move(tmp);
        throw;
      }
#else
      ::new (valptr()) T(std::forward<U>(v));
      this->m_has_val = true;
#endif
    }

    return *this;
  }

  template <class G = E,
            detail::enable_if_t<std::is_nothrow_copy_constructible<G>::value &&
                                std::is_assignable<G &, G>::value> * = nullptr>
  // 由 unexpected 拷贝赋值：切换到错误态
  expected &operator=(const unexpected<G> &rhs) {  // 已有错误则赋值，否则销毁 T 改构造 E
    if (!has_value()) {
      err() = rhs;
    } else {
      this->destroy_val();
      ::new (errptr()) unexpected<E>(rhs);
      this->m_has_val = false;
    }

    return *this;
  }

  template <class G = E,
            detail::enable_if_t<std::is_nothrow_move_constructible<G>::value &&
                                std::is_move_assignable<G>::value> * = nullptr>
  expected &operator=(unexpected<G> &&rhs) noexcept {  // 移动版错误赋值
    if (!has_value()) {
      err() = std::move(rhs);
    } else {
      this->destroy_val();
      ::new (errptr()) unexpected<E>(std::move(rhs));
      this->m_has_val = false;
    }

    return *this;
  }

  template <class... Args, detail::enable_if_t<std::is_nothrow_constructible<
                               T, Args &&...>::value> * = nullptr>
  // emplace：原地重建成功值。重载 A：T 可 noexcept 构造，直接销毁旧内容后 placement-new
  void emplace(Args &&...args) {  // 有值先析构 T；无值析构 E 并翻状态；随后构造新 T
    if (has_value()) {
      val().~T();
    } else {
      err().~unexpected<E>();
      this->m_has_val = true;
    }
    ::new (valptr()) T(std::forward<Args>(args)...);
  }

  template <class... Args, detail::enable_if_t<!std::is_nothrow_constructible<
                               T, Args &&...>::value> * = nullptr>
  // emplace 重载 B：T 构造可能抛，需要临时保存旧 E 以便回滚
  void emplace(Args &&...args) {
    if (has_value()) {
      val().~T();
      ::new (valptr()) T(std::forward<Args>(args)...);
    } else {
      auto tmp = std::move(err());
      err().~unexpected<E>();

#ifdef TL_EXPECTED_EXCEPTIONS_ENABLED
      try {
        ::new (valptr()) T(std::forward<Args>(args)...);
        this->m_has_val = true;
      } catch (...) {
        err() = std::move(tmp);
        throw;
      }
#else
      ::new (valptr()) T(std::forward<Args>(args)...);
      this->m_has_val = true;
#endif
    }
  }

  template <class U, class... Args,
            detail::enable_if_t<std::is_nothrow_constructible<
                T, std::initializer_list<U> &, Args &&...>::value> * = nullptr>
  // emplace 初始化列表版 A(noexcept 构造)：有值时先构造局部 T 再移动赋值，无值直接 placement-new
  void emplace(std::initializer_list<U> il, Args &&...args) {
    if (has_value()) {
      T t(il, std::forward<Args>(args)...);
      val() = std::move(t);
    } else {
      err().~unexpected<E>();
      ::new (valptr()) T(il, std::forward<Args>(args)...);
      this->m_has_val = true;
    }
  }

  template <class U, class... Args,
            detail::enable_if_t<!std::is_nothrow_constructible<
                T, std::initializer_list<U> &, Args &&...>::value> * = nullptr>
  // emplace 初始化列表版 B(可能抛)：同样带保存-回滚保护
  void emplace(std::initializer_list<U> il, Args &&...args) {
    if (has_value()) {
      T t(il, std::forward<Args>(args)...);
      val() = std::move(t);
    } else {
      auto tmp = std::move(err());
      err().~unexpected<E>();

#ifdef TL_EXPECTED_EXCEPTIONS_ENABLED
      try {
        ::new (valptr()) T(il, std::forward<Args>(args)...);
        this->m_has_val = true;
      } catch (...) {
        err() = std::move(tmp);
        throw;
      }
#else
      ::new (valptr()) T(il, std::forward<Args>(args)...);
      this->m_has_val = true;
#endif
    }
  }
private:
  /*===========================================================================
 * 第九部分：swap 实现(标签分派)与观察者接口
 * 用一组空标签类型在编译期选择 T=void / 是否 noexcept 移动构造 的不同交换策略。
 *===========================================================================*/
  using t_is_void = std::true_type;  // 标签：T 是 void
  using t_is_not_void = std::false_type;  // 标签：T 非 void
  using t_is_nothrow_move_constructible = std::true_type;  // 标签：T 可 noexcept 移动构造
  using move_constructing_t_can_throw = std::false_type;  // 标签：移动构造 T 会抛
  using e_is_nothrow_move_constructible = std::true_type;  // 标签：E 可 noexcept 移动构造
  using move_constructing_e_can_throw = std::false_type;  // 标签：移动构造 E 会抛

  void swap_where_both_have_value(expected & /*rhs*/, t_is_void) noexcept {  // 双方都有值且 T=void：无物可换，空操作
    // swapping void is a no-op
  }

  void swap_where_both_have_value(expected &rhs, t_is_not_void) {  // 双方都有值且 T 非 void：普通 swap 两个 T
    using std::swap;  // 双方都无值：交换两个错误对象
    swap(val(), rhs.val());
  }

  void swap_where_only_one_has_value(expected &rhs, t_is_void) noexcept(  // 仅一方有值且 T=void：把对方 E 搬过来、销毁对方 E、互换状态标志
      std::is_nothrow_move_constructible<E>::value) {
    ::new (errptr()) unexpected_type(std::move(rhs.err()));
    rhs.err().~unexpected_type();
    std::swap(this->m_has_val, rhs.m_has_val);
  }

  void swap_where_only_one_has_value(expected &rhs, t_is_not_void) {  // T 非 void：再按 T/E 移动构造是否抛，分派到下面两个实现
    swap_where_only_one_has_value_and_t_is_not_void(
        rhs, typename std::is_nothrow_move_constructible<T>::type{},
        typename std::is_nothrow_move_constructible<E>::type{});
  }

  // 情形A：T、E 都能 noexcept 移动 —— 经临时量完成“值<->错”状态互换，全程不抛
  void swap_where_only_one_has_value_and_t_is_not_void(
      expected &rhs, t_is_nothrow_move_constructible,
      e_is_nothrow_move_constructible) noexcept {
    auto temp = std::move(val());
    val().~T();
    ::new (errptr()) unexpected_type(std::move(rhs.err()));
    rhs.err().~unexpected_type();
    ::new (rhs.valptr()) T(std::move(temp));
    std::swap(this->m_has_val, rhs.m_has_val);
  }

  // 情形B：移动构造 E 会抛 —— 出异常时把临时 T 还原回 *this
  void swap_where_only_one_has_value_and_t_is_not_void(
      expected &rhs, t_is_nothrow_move_constructible,
      move_constructing_e_can_throw) {
    auto temp = std::move(val());
    val().~T();
#ifdef TL_EXPECTED_EXCEPTIONS_ENABLED
    try {
      ::new (errptr()) unexpected_type(std::move(rhs.err()));
      rhs.err().~unexpected_type();
      ::new (rhs.valptr()) T(std::move(temp));
      std::swap(this->m_has_val, rhs.m_has_val);
    } catch (...) {
      val() = std::move(temp);
      throw;
    }
#else
    ::new (errptr()) unexpected_type(std::move(rhs.err()));
    rhs.err().~unexpected_type();
    ::new (rhs.valptr()) T(std::move(temp));
    std::swap(this->m_has_val, rhs.m_has_val);
#endif
  }

  // 情形C：移动构造 T 会抛 —— 反向搬运，出异常时把 E 还原回 rhs
  void swap_where_only_one_has_value_and_t_is_not_void(
      expected &rhs, move_constructing_t_can_throw,
      e_is_nothrow_move_constructible) {
    auto temp = std::move(rhs.err());
    rhs.err().~unexpected_type();
#ifdef TL_EXPECTED_EXCEPTIONS_ENABLED
    try {
      ::new (rhs.valptr()) T(std::move(val()));
      val().~T();
      ::new (errptr()) unexpected_type(std::move(temp));
      std::swap(this->m_has_val, rhs.m_has_val);
    } catch (...) {
      rhs.err() = std::move(temp);
      throw;
    }
#else
    ::new (rhs.valptr()) T(std::move(val()));
    val().~T();
    ::new (errptr()) unexpected_type(std::move(temp));
    std::swap(this->m_has_val, rhs.m_has_val);
#endif
  }

public:
  template <class OT = T, class OE = E>
  detail::enable_if_t<detail::is_swappable<OT>::value &&
                      detail::is_swappable<OE>::value &&
                      (std::is_nothrow_move_constructible<OT>::value ||
                       std::is_nothrow_move_constructible<OE>::value)>
  // 对外 swap：按四种状态组合分派(都有值/仅rhs有值/仅*this有值/都无值)；
  // noexcept 条件综合了 T、E 的移动构造与可交换性
  swap(expected &rhs) noexcept(
      std::is_nothrow_move_constructible<T>::value
          &&detail::is_nothrow_swappable<T>::value
              &&std::is_nothrow_move_constructible<E>::value
                  &&detail::is_nothrow_swappable<E>::value) {
    if (has_value() && rhs.has_value()) {
      swap_where_both_have_value(rhs, typename std::is_void<T>::type{});
    } else if (!has_value() && rhs.has_value()) {  // 仅 rhs 有值：反过来调用 rhs.swap(*this) 复用逻辑
      rhs.swap(*this);
    } else if (has_value()) {  // 仅 *this 有值：走单方有值的标签分派
      swap_where_only_one_has_value(rhs, typename std::is_void<T>::type{});
    } else {
      using std::swap;
      swap(err(), rhs.err());
    }
  }

  // ---- 观察者接口：->、*、has_value、bool、value、error、value_or ----
  constexpr const T *operator->() const {
    TL_ASSERT(has_value());
    return valptr();
  }
  TL_EXPECTED_11_CONSTEXPR T *operator->() {  // -> 运算符(先断言有值)
    TL_ASSERT(has_value());
    return valptr();
  }

  template <class U = T, detail::enable_if_t<!std::is_void<U>::value> * = nullptr>
  constexpr const U &operator*() const & {  // 解引用 const 左值：取 const T&
    TL_ASSERT(has_value());
    return val();
  }
  template <class U = T, detail::enable_if_t<!std::is_void<U>::value> * = nullptr>
  TL_EXPECTED_11_CONSTEXPR U &operator*() & {  // 解引用左值：取 T&
    TL_ASSERT(has_value());
    return val();
  }
  template <class U = T, detail::enable_if_t<!std::is_void<U>::value> * = nullptr>
  constexpr const U &&operator*() const && {  // 解引用 const 右值
    TL_ASSERT(has_value());
    return std::move(val());
  }
  template <class U = T, detail::enable_if_t<!std::is_void<U>::value> * = nullptr>
  TL_EXPECTED_11_CONSTEXPR U &&operator*() && {  // 解引用右值：移动取 T
    TL_ASSERT(has_value());
    return std::move(val());
  }

  constexpr bool has_value() const noexcept { return this->m_has_val; }  // 是否有值(noexcept)

  constexpr explicit operator bool() const noexcept { return this->m_has_val; }  // 显式 bool 转换，等价 has_value()

  template <class U = T, detail::enable_if_t<!std::is_void<U>::value> * = nullptr>
  // value()：有值返回 T，无值抛 bad_expected_access(禁用异常时 terminate)；四种引用限定符
  TL_EXPECTED_11_CONSTEXPR const U &value() const & {
    if (!has_value())
      TL_EXPECTED_THROW_EXCEPTION(bad_expected_access<E>(err().value()));
    return val();
  }
  template <class U = T, detail::enable_if_t<!std::is_void<U>::value> * = nullptr>
  TL_EXPECTED_11_CONSTEXPR U &value() & {  // 可变左值版 value()
    if (!has_value())
      TL_EXPECTED_THROW_EXCEPTION(bad_expected_access<E>(err().value()));
    return val();
  }
  template <class U = T, detail::enable_if_t<!std::is_void<U>::value> * = nullptr>
  TL_EXPECTED_11_CONSTEXPR const U &&value() const && {  // const 右值版 value()
    if (!has_value())
      TL_EXPECTED_THROW_EXCEPTION(
          bad_expected_access<E>(std::move(err()).value()));
    return std::move(val());
  }
  template <class U = T, detail::enable_if_t<!std::is_void<U>::value> * = nullptr>
  TL_EXPECTED_11_CONSTEXPR U &&value() && {  // 右值版 value()
    if (!has_value())
      TL_EXPECTED_THROW_EXCEPTION(
          bad_expected_access<E>(std::move(err()).value()));
    return std::move(val());
  }

  // error()：与 value() 相反，断言“无值”后返回错误 E
  constexpr const E &error() const & {
    TL_ASSERT(!has_value());
    return err().value();
  }
  TL_EXPECTED_11_CONSTEXPR E &error() & {  // 可变左值取错误
    TL_ASSERT(!has_value());
    return err().value();
  }
  constexpr const E &&error() const && {  // const 右值取错误
    TL_ASSERT(!has_value());
    return std::move(err().value());
  }
  TL_EXPECTED_11_CONSTEXPR E &&error() && {  // 右值移动取错误
    TL_ASSERT(!has_value());
    return std::move(err().value());
  }

  template <class U>
  // value_or：有值返回值，无值返回给定的兜底 v(静态断言 T 可拷贝构造且 U 可转 T)
  constexpr T value_or(U &&v) const & {  // const 左值版
    static_assert(std::is_copy_constructible<T>::value &&
                      std::is_convertible<U &&, T>::value,
                  "T must be copy-constructible and convertible to from U&&");
    return bool(*this) ? **this : static_cast<T>(std::forward<U>(v));
  }
  template <class U>
  TL_EXPECTED_11_CONSTEXPR T value_or(U &&v) && {  // 右值版：有值则移动取出，否则用兜底 v
    static_assert(std::is_move_constructible<T>::value &&
                      std::is_convertible<U &&, T>::value,
                  "T must be move-constructible and convertible to from U&&");
    return bool(*this) ? std::move(**this)
                       : static_cast<T>(std::forward<U>(v));
  }
};
namespace detail {
template <class Exp>
/*===========================================================================
 * 第十部分：单子接口的实现细节(and_then/map/map_error/or_else 的 _impl)
 * 每个都同时处理“输入 expected 的 T 是否为 void”“回调返回是否为 void”，
 * 并在 C++14(返回值推导) 与 C++11(尾置返回类型) 下各写一套等价实现。
 *===========================================================================*/
using exp_t = typename detail::decay_t<Exp>::value_type;  // 取出输入 Exp 的成功值类型 T
template <class Exp>
using err_t = typename detail::decay_t<Exp>::error_type;  // 取出输入 Exp 的错误类型 E
template <class Exp, class Ret>
using ret_t = expected<Ret, err_t<Exp>>;  // 回调返回 Ret 时，整体结果类型是 expected<Ret, 原错误类型>

#ifdef TL_EXPECTED_CXX14
template <class Exp, class F,
          detail::enable_if_t<!std::is_void<exp_t<Exp>>::value> * = nullptr,
          class Ret = decltype(detail::invoke(std::declval<F>(),
                                              *std::declval<Exp>()))>
// and_then 实现(T 非 void)：静态断言 f 返回 expected；有值把 *exp 传给 f，无值用原错误构造结果
constexpr auto and_then_impl(Exp &&exp, F &&f) {
  static_assert(detail::is_expected<Ret>::value, "F must return an expected");

  return exp.has_value()
             ? detail::invoke(std::forward<F>(f), *std::forward<Exp>(exp))
             : Ret(unexpect, std::forward<Exp>(exp).error());
}

template <class Exp, class F,
          detail::enable_if_t<std::is_void<exp_t<Exp>>::value> * = nullptr,
          class Ret = decltype(detail::invoke(std::declval<F>()))>
// and_then 实现(T=void)：有值时 f 不接收参数
constexpr auto and_then_impl(Exp &&exp, F &&f) {
  static_assert(detail::is_expected<Ret>::value, "F must return an expected");

  return exp.has_value() ? detail::invoke(std::forward<F>(f))
                         : Ret(unexpect, std::forward<Exp>(exp).error());
}
#else
template <class>
struct TC;  // (原代码保留的调试用未完成类型声明，未使用)
template <class Exp, class F,
          class Ret = decltype(detail::invoke(std::declval<F>(),
                                              *std::declval<Exp>())),
          detail::enable_if_t<!std::is_void<exp_t<Exp>>::value> * = nullptr>
//   ^— 以上为 C++14 版；以下是 C++11 尾置返回类型的等价实现 —^
auto and_then_impl(Exp &&exp, F &&f) -> Ret {
  static_assert(detail::is_expected<Ret>::value, "F must return an expected");

  return exp.has_value()
             ? detail::invoke(std::forward<F>(f), *std::forward<Exp>(exp))
             : Ret(unexpect, std::forward<Exp>(exp).error());
}

template <class Exp, class F,
          class Ret = decltype(detail::invoke(std::declval<F>())),
          detail::enable_if_t<std::is_void<exp_t<Exp>>::value> * = nullptr>
constexpr auto and_then_impl(Exp &&exp, F &&f) -> Ret {  // C++11 版 and_then(T=void)
  static_assert(detail::is_expected<Ret>::value, "F must return an expected");

  return exp.has_value() ? detail::invoke(std::forward<F>(f))
                         : Ret(unexpect, std::forward<Exp>(exp).error());
}
#endif

#ifdef TL_EXPECTED_CXX14
template <class Exp, class F,
          detail::enable_if_t<!std::is_void<exp_t<Exp>>::value> * = nullptr,
          class Ret = decltype(detail::invoke(std::declval<F>(),
                                              *std::declval<Exp>())),
          detail::enable_if_t<!std::is_void<Ret>::value> * = nullptr>
// map 实现：输入 T 非 void、回调返回非 void —— 有值对 *exp 调 f 并包成 result，无值透传错误
constexpr auto expected_map_impl(Exp &&exp, F &&f) {
  using result = ret_t<Exp, detail::decay_t<Ret>>;
  return exp.has_value() ? result(detail::invoke(std::forward<F>(f),
                                                 *std::forward<Exp>(exp)))
                         : result(unexpect, std::forward<Exp>(exp).error());
}

template <class Exp, class F,
          detail::enable_if_t<!std::is_void<exp_t<Exp>>::value> * = nullptr,
          class Ret = decltype(detail::invoke(std::declval<F>(),
                                              *std::declval<Exp>())),
          detail::enable_if_t<std::is_void<Ret>::value> * = nullptr>
// map 实现：输入 T 非 void、回调返回 void —— 有值仅执行副作用 f，结果为 expected<void,E>
auto expected_map_impl(Exp &&exp, F &&f) {
  using result = expected<void, err_t<Exp>>;
  if (exp.has_value()) {
    detail::invoke(std::forward<F>(f), *std::forward<Exp>(exp));
    return result();
  }

  return result(unexpect, std::forward<Exp>(exp).error());
}

template <class Exp, class F,
          detail::enable_if_t<std::is_void<exp_t<Exp>>::value> * = nullptr,
          class Ret = decltype(detail::invoke(std::declval<F>())),
          detail::enable_if_t<!std::is_void<Ret>::value> * = nullptr>
// map 实现：输入 T=void、回调返回非 void —— 有值调用无参 f 并包装其结果
constexpr auto expected_map_impl(Exp &&exp, F &&f) {
  using result = ret_t<Exp, detail::decay_t<Ret>>;
  return exp.has_value() ? result(detail::invoke(std::forward<F>(f)))
                         : result(unexpect, std::forward<Exp>(exp).error());
}

template <class Exp, class F,
          detail::enable_if_t<std::is_void<exp_t<Exp>>::value> * = nullptr,
          class Ret = decltype(detail::invoke(std::declval<F>())),
          detail::enable_if_t<std::is_void<Ret>::value> * = nullptr>
// map 实现：输入 T=void、回调也返回 void —— 仅执行 f，结果 expected<void,E>
auto expected_map_impl(Exp &&exp, F &&f) {
  using result = expected<void, err_t<Exp>>;
  if (exp.has_value()) {
    detail::invoke(std::forward<F>(f));
    return result();
  }

  return result(unexpect, std::forward<Exp>(exp).error());
}
#else
template <class Exp, class F,
          detail::enable_if_t<!std::is_void<exp_t<Exp>>::value> * = nullptr,
          class Ret = decltype(detail::invoke(std::declval<F>(),
                                              *std::declval<Exp>())),
          detail::enable_if_t<!std::is_void<Ret>::value> * = nullptr>

//   ^— C++14 版结束；以下为 C++11 尾置返回版 map 的四种组合 —^
constexpr auto expected_map_impl(Exp &&exp, F &&f)
    -> ret_t<Exp, detail::decay_t<Ret>> {
  using result = ret_t<Exp, detail::decay_t<Ret>>;

  return exp.has_value() ? result(detail::invoke(std::forward<F>(f),
                                                 *std::forward<Exp>(exp)))
                         : result(unexpect, std::forward<Exp>(exp).error());
}

template <class Exp, class F,
          detail::enable_if_t<!std::is_void<exp_t<Exp>>::value> * = nullptr,
          class Ret = decltype(detail::invoke(std::declval<F>(),
                                              *std::declval<Exp>())),
          detail::enable_if_t<std::is_void<Ret>::value> * = nullptr>

auto expected_map_impl(Exp &&exp, F &&f) -> expected<void, err_t<Exp>> {  // C++11 版：回调返回 void，结果 expected<void,E>
  if (exp.has_value()) {
    detail::invoke(std::forward<F>(f), *std::forward<Exp>(exp));
    return {};
  }

  return unexpected<err_t<Exp>>(std::forward<Exp>(exp).error());
}

template <class Exp, class F,
          detail::enable_if_t<std::is_void<exp_t<Exp>>::value> * = nullptr,
          class Ret = decltype(detail::invoke(std::declval<F>())),
          detail::enable_if_t<!std::is_void<Ret>::value> * = nullptr>

constexpr auto expected_map_impl(Exp &&exp, F &&f)
    -> ret_t<Exp, detail::decay_t<Ret>> {
  using result = ret_t<Exp, detail::decay_t<Ret>>;

  return exp.has_value() ? result(detail::invoke(std::forward<F>(f)))
                         : result(unexpect, std::forward<Exp>(exp).error());
}

template <class Exp, class F,
          detail::enable_if_t<std::is_void<exp_t<Exp>>::value> * = nullptr,
          class Ret = decltype(detail::invoke(std::declval<F>())),
          detail::enable_if_t<std::is_void<Ret>::value> * = nullptr>

auto expected_map_impl(Exp &&exp, F &&f) -> expected<void, err_t<Exp>> {  // C++11 版：输入/回调均 void
  if (exp.has_value()) {
    detail::invoke(std::forward<F>(f));
    return {};
  }

  return unexpected<err_t<Exp>>(std::forward<Exp>(exp).error());
}
#endif

#if defined(TL_EXPECTED_CXX14) && !defined(TL_EXPECTED_GCC49) &&               \
    !defined(TL_EXPECTED_GCC54) && !defined(TL_EXPECTED_GCC55)
template <class Exp, class F,
          detail::enable_if_t<!std::is_void<exp_t<Exp>>::value> * = nullptr,
          class Ret = decltype(detail::invoke(std::declval<F>(),
                                              std::declval<Exp>().error())),
          detail::enable_if_t<!std::is_void<Ret>::value> * = nullptr>
// map_error 实现：只在错误态对 error 调 f。输入 T 非 void、回调返回非 void -> 新错误类型
constexpr auto map_error_impl(Exp &&exp, F &&f) {
  using result = expected<exp_t<Exp>, detail::decay_t<Ret>>;
  return exp.has_value()
             ? result(*std::forward<Exp>(exp))
             : result(unexpect, detail::invoke(std::forward<F>(f),
                                               std::forward<Exp>(exp).error()));
}
template <class Exp, class F,
          detail::enable_if_t<!std::is_void<exp_t<Exp>>::value> * = nullptr,
          class Ret = decltype(detail::invoke(std::declval<F>(),
                                              std::declval<Exp>().error())),
          detail::enable_if_t<std::is_void<Ret>::value> * = nullptr>
// map_error：输入 T 非 void、回调返回 void(仅副作用) -> 错误类型用 monostate 占位
auto map_error_impl(Exp &&exp, F &&f) {
  using result = expected<exp_t<Exp>, monostate>;
  if (exp.has_value()) {
    return result(*std::forward<Exp>(exp));
  }

  detail::invoke(std::forward<F>(f), std::forward<Exp>(exp).error());
  return result(unexpect, monostate{});
}
template <class Exp, class F,
          detail::enable_if_t<std::is_void<exp_t<Exp>>::value> * = nullptr,
          class Ret = decltype(detail::invoke(std::declval<F>(),
                                              std::declval<Exp>().error())),
          detail::enable_if_t<!std::is_void<Ret>::value> * = nullptr>
// map_error：输入 T=void、回调返回非 void
constexpr auto map_error_impl(Exp &&exp, F &&f) {
  using result = expected<exp_t<Exp>, detail::decay_t<Ret>>;
  return exp.has_value()
             ? result()
             : result(unexpect, detail::invoke(std::forward<F>(f),
                                               std::forward<Exp>(exp).error()));
}
template <class Exp, class F,
          detail::enable_if_t<std::is_void<exp_t<Exp>>::value> * = nullptr,
          class Ret = decltype(detail::invoke(std::declval<F>(),
                                              std::declval<Exp>().error())),
          detail::enable_if_t<std::is_void<Ret>::value> * = nullptr>
// map_error：输入 T=void、回调返回 void
auto map_error_impl(Exp &&exp, F &&f) {
  using result = expected<exp_t<Exp>, monostate>;
  if (exp.has_value()) {
    return result();
  }

  detail::invoke(std::forward<F>(f), std::forward<Exp>(exp).error());
  return result(unexpect, monostate{});
}
#else
template <class Exp, class F,
          detail::enable_if_t<!std::is_void<exp_t<Exp>>::value> * = nullptr,
          class Ret = decltype(detail::invoke(std::declval<F>(),
                                              std::declval<Exp>().error())),
          detail::enable_if_t<!std::is_void<Ret>::value> * = nullptr>
//  ^— C++14 版结束；以下为 C++11 尾置返回版 map_error 四组合 —^
constexpr auto map_error_impl(Exp &&exp, F &&f)
    -> expected<exp_t<Exp>, detail::decay_t<Ret>> {
  using result = expected<exp_t<Exp>, detail::decay_t<Ret>>;

  return exp.has_value()
             ? result(*std::forward<Exp>(exp))
             : result(unexpect, detail::invoke(std::forward<F>(f),
                                               std::forward<Exp>(exp).error()));
}

template <class Exp, class F,
          detail::enable_if_t<!std::is_void<exp_t<Exp>>::value> * = nullptr,
          class Ret = decltype(detail::invoke(std::declval<F>(),
                                              std::declval<Exp>().error())),
          detail::enable_if_t<std::is_void<Ret>::value> * = nullptr>
auto map_error_impl(Exp &&exp, F &&f) -> expected<exp_t<Exp>, monostate> {  // C++11 版：回调 void，错误占位 monostate
  using result = expected<exp_t<Exp>, monostate>;
  if (exp.has_value()) {
    return result(*std::forward<Exp>(exp));
  }

  detail::invoke(std::forward<F>(f), std::forward<Exp>(exp).error());
  return result(unexpect, monostate{});
}

template <class Exp, class F,
          detail::enable_if_t<std::is_void<exp_t<Exp>>::value> * = nullptr,
          class Ret = decltype(detail::invoke(std::declval<F>(),
                                              std::declval<Exp>().error())),
          detail::enable_if_t<!std::is_void<Ret>::value> * = nullptr>
constexpr auto map_error_impl(Exp &&exp, F &&f)
    -> expected<exp_t<Exp>, detail::decay_t<Ret>> {
  using result = expected<exp_t<Exp>, detail::decay_t<Ret>>;

  return exp.has_value()
             ? result()
             : result(unexpect, detail::invoke(std::forward<F>(f),
                                               std::forward<Exp>(exp).error()));
}

template <class Exp, class F,
          detail::enable_if_t<std::is_void<exp_t<Exp>>::value> * = nullptr,
          class Ret = decltype(detail::invoke(std::declval<F>(),
                                              std::declval<Exp>().error())),
          detail::enable_if_t<std::is_void<Ret>::value> * = nullptr>
auto map_error_impl(Exp &&exp, F &&f) -> expected<exp_t<Exp>, monostate> {  // C++11 版：输入 void、回调 void
  using result = expected<exp_t<Exp>, monostate>;
  if (exp.has_value()) {
    return result();
  }

  detail::invoke(std::forward<F>(f), std::forward<Exp>(exp).error());
  return result(unexpect, monostate{});
}
#endif

#ifdef TL_EXPECTED_CXX14
template <class Exp, class F,
          class Ret = decltype(detail::invoke(std::declval<F>(),
                                              std::declval<Exp>().error())),
          detail::enable_if_t<!std::is_void<Ret>::value> * = nullptr>
// or_else 实现(回调返回 expected)：有值原样返回 exp；无值调用 f(error) 进行恢复
constexpr auto or_else_impl(Exp &&exp, F &&f) {
  static_assert(detail::is_expected<Ret>::value, "F must return an expected");
  return exp.has_value()
             ? std::forward<Exp>(exp)
             : detail::invoke(std::forward<F>(f),
                              std::forward<Exp>(exp).error());
}

template <class Exp, class F,
          class Ret = decltype(detail::invoke(std::declval<F>(),
                                              std::declval<Exp>().error())),
          detail::enable_if_t<std::is_void<Ret>::value> * = nullptr>
// or_else 实现(回调返回 void，仅副作用)：逗号表达式先执行 f(error)，再把 exp 原样返回
detail::decay_t<Exp> or_else_impl(Exp &&exp, F &&f) {
  return exp.has_value()
             ? std::forward<Exp>(exp)
             : (detail::invoke(std::forward<F>(f),
                               std::forward<Exp>(exp).error()),
                std::forward<Exp>(exp));
}
#else
template <class Exp, class F,
          class Ret = decltype(detail::invoke(std::declval<F>(),
                                              std::declval<Exp>().error())),
          detail::enable_if_t<!std::is_void<Ret>::value> * = nullptr>
//  ^— C++14 版结束；以下 C++11 尾置返回版 or_else —^
auto or_else_impl(Exp &&exp, F &&f) -> Ret {
  static_assert(detail::is_expected<Ret>::value, "F must return an expected");
  return exp.has_value()
             ? std::forward<Exp>(exp)
             : detail::invoke(std::forward<F>(f),
                              std::forward<Exp>(exp).error());
}

template <class Exp, class F,
          class Ret = decltype(detail::invoke(std::declval<F>(),
                                              std::declval<Exp>().error())),
          detail::enable_if_t<std::is_void<Ret>::value> * = nullptr>
detail::decay_t<Exp> or_else_impl(Exp &&exp, F &&f) {  // C++11 版：回调 void 的 or_else
  return exp.has_value()
             ? std::forward<Exp>(exp)
             : (detail::invoke(std::forward<F>(f),
                               std::forward<Exp>(exp).error()),
                std::forward<Exp>(exp));
}
#endif
} // namespace detail
template <class T, class E, class U, class F>
/*===========================================================================
 * 第十一部分：expected 的比较运算符与自由 swap
 *===========================================================================*/
// expected 与 expected(泛型 T)：状态不同即不等；都无值比错误，都有值比值
constexpr bool operator==(const expected<T, E> &lhs,
                          const expected<U, F> &rhs) {
  return (lhs.has_value() != rhs.has_value())
             ? false
             : (!lhs.has_value() ? lhs.error() == rhs.error() : *lhs == *rhs);
}
template <class T, class E, class U, class F>
constexpr bool operator!=(const expected<T, E> &lhs,  // !=：状态不同即不等，否则按值/错误比较
                          const expected<U, F> &rhs) {
  return (lhs.has_value() != rhs.has_value())
             ? true
             : (!lhs.has_value() ? lhs.error() != rhs.error() : *lhs != *rhs);
}
template <class E, class F>
// expected<void> 之间比较：都有值时直接为 true(没有值可比)
constexpr bool operator==(const expected<void, E> &lhs,
                          const expected<void, F> &rhs) {
  return (lhs.has_value() != rhs.has_value())
             ? false
             : (!lhs.has_value() ? lhs.error() == rhs.error() : true);
}
template <class E, class F>
constexpr bool operator!=(const expected<void, E> &lhs,  // void 版 !=
                          const expected<void, F> &rhs) {
  return (lhs.has_value() != rhs.has_value())
             ? true
             : (!lhs.has_value() ? lhs.error() != rhs.error() : false);
}

template <class T, class E, class U>
// expected 与裸值 v：有值才用 *x 与 v 比较(两个参数顺序各重载一次)
constexpr bool operator==(const expected<T, E> &x, const U &v) {
  return x.has_value() ? *x == v : false;
}
template <class T, class E, class U>
constexpr bool operator==(const U &v, const expected<T, E> &x) {  // 左右参数互换的对称重载
  return x.has_value() ? *x == v : false;
}
template <class T, class E, class U>
constexpr bool operator!=(const expected<T, E> &x, const U &v) {  // 与裸值不等
  return x.has_value() ? *x != v : true;
}
template <class T, class E, class U>
constexpr bool operator!=(const U &v, const expected<T, E> &x) {  // 与裸值不等(对称)
  return x.has_value() ? *x != v : true;
}

template <class T, class E>
// expected 与 unexpected：仅当 expected 处于错误态且错误值相等才相等(对称两版)
constexpr bool operator==(const expected<T, E> &x, const unexpected<E> &e) {
  return x.has_value() ? false : x.error() == e.value();
}
template <class T, class E>
constexpr bool operator==(const unexpected<E> &e, const expected<T, E> &x) {  // 对称重载
  return x.has_value() ? false : x.error() == e.value();
}
template <class T, class E>
constexpr bool operator!=(const expected<T, E> &x, const unexpected<E> &e) {  // 与 unexpected 不等
  return x.has_value() ? true : x.error() != e.value();
}
template <class T, class E>
constexpr bool operator!=(const unexpected<E> &e, const expected<T, E> &x) {  // 对称重载
  return x.has_value() ? true : x.error() != e.value();
}

template <class T, class E,
          detail::enable_if_t<(std::is_void<T>::value ||
                               std::is_move_constructible<T>::value) &&
                              detail::is_swappable<T>::value &&
                              std::is_move_constructible<E>::value &&
                              detail::is_swappable<E>::value> * = nullptr>
// 自由 swap：仅当 T(void 或可移动构造)、T/E 都可交换且至少一方可 noexcept 移动时启用，委托成员 swap
void swap(expected<T, E> &lhs,
          expected<T, E> &rhs) noexcept(noexcept(lhs.swap(rhs))) {
  lhs.swap(rhs);
}
} // namespace tl

#endif