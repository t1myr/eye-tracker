#pragma once

#include <type_traits>


template <typename U, typename... Rest>
struct all_same;

template <typename U>
struct all_same<U> : std::true_type {}; // Базовый случай: 1 тип всегда совпадает

template <typename U, typename First, typename... Rest>
struct all_same<U, First, Rest...>
    : std::integral_constant<bool, std::is_same<U, First>::value && all_same<U, Rest...>::value> {};

template<typename... Args>
struct all_unsigned;

template<>
struct all_unsigned<> : std::true_type {};

template<typename First, typename... Rest>
struct all_unsigned<First, Rest...> : std::conditional_t<
    std::is_unsigned<First>::value,
    all_unsigned<Rest...>,
    std::false_type
> {};
