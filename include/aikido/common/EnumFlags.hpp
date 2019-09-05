#ifndef AIKIDO_COMMON_ENUMFLAGS_HPP_
#define AIKIDO_COMMON_ENUMFLAGS_HPP_

#include <type_traits>

/// Enable bitwise operators for strongly-typed enums.

// Adapted from
// http://blog.bitwigglers.org/using-enum-classes-as-type-safe-bitmasks/

// clang-format off

#define AIKIDO_ENABLE_BITWISE_OPERATORS(X)                                     \
template <>                                                                     \
struct EnableBitwiseOperators< X >                                             \
{                                                                              \
  static const bool enable = true;                                             \
};

template <typename Enum>
struct EnableBitwiseOperators
{
  static const bool enable = false;
};

template <typename Enum>
typename std::enable_if<EnableBitwiseOperators<Enum>::enable, Enum>::type
operator&(Enum lhs, Enum rhs)
{
  using T = typename std::underlying_type<Enum>::type;
  return static_cast<Enum>(static_cast<T>(lhs) & static_cast<T>(rhs));
}

template <typename Enum>
typename std::enable_if<EnableBitwiseOperators<Enum>::enable, Enum>::type
operator|(Enum lhs, Enum rhs)
{
  using T = typename std::underlying_type<Enum>::type;
  return static_cast<Enum>(static_cast<T>(lhs) | static_cast<T>(rhs));
}

// clang-format on

#endif // AIKIDO_COMMON_ENUMFLAGS_HPP_
