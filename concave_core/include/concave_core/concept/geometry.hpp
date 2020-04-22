/******************************************************************************
 * \author      Emelyanov Dmitry <dmitriy.emelyanov.de@gmail.com>
 *
 * \brief       A concept that defines the required properties for a type
 ******************************************************************************/

#ifndef CONCAVE_GEOMETRY_HPP
#define CONCAVE_GEOMETRY_HPP

#include "concave_core/type_traits/has_coordinates.hpp"

namespace concave {

template<typename T, typename = std::enable_if_t<has_coordinates_v<std::decay_t<T>>>>
using Geometry = T;

} // namespace concave

#endif //CONCAVE_GEOMETRY_HPP