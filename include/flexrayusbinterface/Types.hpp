#pragma once

#include <mapbox/variant.hpp>

template <typename... Args>
using variant = mapbox::util::variant<Args...>;
