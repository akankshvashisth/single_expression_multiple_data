

#include <array>
#include <vector>

#define FMT_HEADER_ONLY

#include <fmt/format.h>
#include <fmt/printf.h>
#include <fmt/ranges.h>

#include <string>

#include "semd.hpp"

#define USE_AKS_SIMD

#ifdef USE_AKS_SIMD

#include "simd.hpp"
namespace aks::semd::ops {
template <aks::simd::is_simd_c T,
          aks::simd::is_simd_c U,
          aks::simd::is_simd_c V>
struct select_f<T, U, V> {
  auto operator()(T x, U y, V z) const { return aks::simd::select(x, y, z); }
};
}  // namespace aks::semd::ops

template <typename T>
void simd_check_1d() {
  using vec_t = T;
  using s_t = typename T::scalar_type;
  using mask_t = typename aks::simd::mask<typename T::spec>;

  std::vector<vec_t> const xs{s_t{-1}, s_t{2}, s_t{5}};
  std::array<vec_t, 8> ys{s_t{6}, s_t{1}, s_t{2}};

  aks::semd::semd x{xs};
  aks::semd::semd y{ys};

  auto const mtrue = aks::simd::from_bools<mask_t>(true);
  auto const mfalse = aks::simd::from_bools<mask_t>(false);

  fmt::println(" x = {}", x);
  fmt::println("-x = {}", -x);
  fmt::println(" y = {}", y);

  fmt::println("x + y + x + y = {}", x + y + x + y);
  fmt::println("x[1] = {}", x[1]);
  fmt::println("min  = {}", select(x < y, x, y));

  fmt::println("select(x < y, x, y)  = {}", select(x < y, x, y));
  fmt::println("select(x < y, x, 2)  = {}", select(x < y, x, vec_t{s_t{2}}));
  fmt::println("select(x < y, 2, y)  = {}", select(x < y, vec_t{s_t{2}}, y));
  fmt::println("select(x < y, 3, 2)  = {}",
               select(x < y, vec_t{s_t{3}}, vec_t{s_t{2}}));
  fmt::println("select(mtrue, x, y)  = {}", select(mtrue, x, y));
  fmt::println("select(mfalse, x, 2) = {}", select(mfalse, x, vec_t{s_t{2}}));
  fmt::println("select(mfalse, 3, y) = {}", select(mfalse, vec_t{s_t{3}}, y));
  fmt::println("sqrt(x) * sqrt(x)    = {}", sqrt(x) * sqrt(x));
  fmt::println("2.0 * sqrt(x)        = {}", vec_t{2.0} * sqrt(x));
  fmt::println("sqrt(x) * 2.0        = {}", sqrt(x) * vec_t{2.0});
}

void simd_examples() {
  simd_check_1d<aks::simd::vec1d>();
  simd_check_1d<aks::simd::vec1f>();
  simd_check_1d<aks::simd::vec1i>();

  using aks::simd::vec4d;
  using aks::simd::vec4d_mask;

  std::vector<vec4d> const xs{vec4d{-1., 2., 5., 6.}, vec4d{8., 10., 1.2, 1.5}};
  std::array<vec4d, 2> ys{vec4d{6., 1., 2., 8.}, vec4d{9., 10., 1.1, 1.6}};

  aks::semd::semd x{xs};
  aks::semd::semd y{ys};

  auto mtrue = aks::simd::from_bools<vec4d_mask>(true, true, true, true);
  auto mfalse = aks::simd::from_bools<vec4d_mask>(false, false, false, false);

  fmt::println(" x = {}", x);
  fmt::println("-x = {}", -x);
  fmt::println(" y = {}", y);

  fmt::println("x + y + x + y = {}", x + y + x + y);

  fmt::println("x[1] = {}", x[1]);
  fmt::println("min  = {}", select(x < y, x, y));

  fmt::println("select(x < y, x, y)     = {}", select(x < y, x, y));
  fmt::println("select(x < y, x, 2.0)   = {}", select(x < y, x, vec4d{2.0}));
  fmt::println("select(x < y, 2.0, y)   = {}", select(x < y, vec4d{2.0}, y));

  fmt::println("select(x < y, 3.0, 2.0) = {}",
               select(x < y, vec4d{3.0}, vec4d{2.0}));
  fmt::println("select(mtrue, x, y)     = {}", select(mtrue, x, y));
  fmt::println("select(mfalse, x, 2.0)  = {}", select(mfalse, x, vec4d{2.0}));
  fmt::println("select(mfalse, 3.0, y)  = {}", select(mfalse, vec4d{3.0}, y));

  fmt::println("sqrt(x) * sqrt(x)       = {}", sqrt(x) * sqrt(x));
  fmt::println("2.0 * sqrt(x)           = {}", vec4d{2.0} * sqrt(x));
  fmt::println("sqrt(x) * 2.0           = {}", sqrt(x) * vec4d{2.0});
}

#else
void simd_examples() {}
#endif

namespace aks::semd::ops {
template <>
struct neg_f<std::string> {
  auto operator()(std::string const& x) const {
    return std::string(x.rbegin(), x.rend());
  }
};
}  // namespace aks::semd::ops

int main() {
  simd_examples();
  {
    std::vector<double> xs = {-1.0, 2.0, 3.0, 4.0};
    std::array<float, 4> ys = {1.0, -2.0, 5.0, 0.0};
    std::ranges::repeat_view const two{2.0};

    aks::semd::semd x{xs};
    aks::semd::semd y{ys};
    aks::semd::semd t{two};

    fmt::println(" x  = {}", x);
    fmt::println("-x  = {}", -x);
    fmt::println(" y  = {}", y);

    fmt::println("min = {}", select(x < y, x, y));

    fmt::println("x + y + x + y = {}", x + y + x + y);

    fmt::println("x[2] = {}", x[2]);

    fmt::println("select(!(x < y), x, y)  = {}", select(!(x < y), x, y));
    fmt::println("select(x < y, x, 2.0)   = {}", select(x < y, x, 2.0));
    fmt::println("select(x < y, 2.0, y)   = {}", select(x < y, 2.0, y));
    fmt::println("select(true, x, y)      = {}", select(true, x, y));
    fmt::println("select(x < y, 3.0, 2.0) = {}", select(x < y, 3.0, 2.0));
    fmt::println("select(false, x, 2.0)   = {}", select(false, x, 2.0));
    fmt::println("select(false, 3.0, y)   = {}", select(false, 3.0, y));

    fmt::println("sin(x) * sin(x) + cos(x) * cos(x) = {}",
                 sin(x) * sin(x) + cos(x) * cos(x));
    fmt::println("pow(x, y)                         = {}", pow(x, y));
    fmt::println("pow(x, 2.0)                       = {}", pow(x, 2.0));
    fmt::println("2.0 * pow(x, y)                   = {}", 2.0 * pow(x, y));
    fmt::println("pow(x, y) * 2.0                   = {}", pow(x, y) * 2.0);
  }
  {
    std::vector<std::string> xs{"abc", "bcd", "cde"};
    std::array<std::string, 3> ys{"def", "efg", "fgh"};

    aks::semd::semd x{xs};
    aks::semd::semd y{ys};

    aks::semd::expression ex{[](auto const& s) { return s.size(); }, x};

    fmt::println(" x    = {}", x);
    fmt::println("-x    = {}", -x);
    fmt::println("s*2   = {}", ex * 2.0);
    fmt::println("  y   = {}", y);
    fmt::println("x + y = {}", x + y);

    fmt::println("x[2]  = {}", x[2]);

    fmt::println("min   = {}", select(x < y, x, y));
  }
  return 0;
}
