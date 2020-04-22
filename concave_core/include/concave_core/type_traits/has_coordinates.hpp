/******************************************************************************
 * \author      Emelyanov Dmitry <dmitriy.emelyanov.de@gmail.com>
 *
 * \brief       Checking for fields or functions x and y for the type
 ******************************************************************************/

#ifndef CONCAVE_HAS_COORDINATES_HPP
#define CONCAVE_HAS_COORDINATES_HPP

#include <type_traits>

namespace concave {

template<typename T, typename = void>
struct has_coordinates : std::false_type {
};

template<typename T>
struct has_coordinates<T, typename std::enable_if_t<std::is_member_pointer_v<decltype(&T::x)>
                                                    && std::is_member_pointer_v<decltype(&T::y)>>> : std::true_type {
};

template<typename T>
inline constexpr bool has_coordinates_v = has_coordinates<T>::value;

} // namespace concave

#endif // CONCAVE_HAS_COORDINATES_HPP