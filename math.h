#ifndef MATH_H
#define MATH_H

#include <type_traits>
//#include <core/math/math_defs.h>

// https://stackoverflow.com/posts/4609795/
// written by Stef: https://stackoverflow.com/users/3080723/stef
template <typename T> inline constexpr
int signum(T x, std::false_type is_signed) {
    return T(0) < x;
}

template <typename T> inline constexpr
int signum(T x, std::true_type is_signed) {
    return (T(0) < x) - (x < T(0));
}

template <typename T> inline constexpr
int signum(T x) {
    return signum(x, std::is_signed<T>());
}

#endif // MATH_H
