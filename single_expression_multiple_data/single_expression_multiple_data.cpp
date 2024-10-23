

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
  auto operator()(T x, U y, V z) const {
    auto res = aks::simd::select(x, y, z);
    // fmt::println("x = {}\ny = {}\nz = {}\nres = {}", x, y, z, res);
    return res;
  }
};
}  // namespace aks::semd::ops

template <typename T, typename U>
void simd_check(U const filter01, U const filter02) {
  using vec_t = T;
  using s_t = typename T::scalar_type;
  using mask_t = typename aks::simd::mask<typename T::spec>;

  std::vector<vec_t> const xs{vec_t{s_t{-1}}, vec_t{s_t{2}}, vec_t{s_t{5}}};
  std::array<vec_t, 3> ys{vec_t{s_t{6}}, vec_t{s_t{1}}, vec_t{s_t{2}}};

  aks::semd::semd x{xs};
  aks::semd::semd y{ys};

  fmt::println(" x = {}", x);
  fmt::println("-x = {}", -x);
  fmt::println(" y = {}", y);

  fmt::println("x + y + x + y = {}", x + y + x + y);
  fmt::println("x[1] = {}", x[1]);
  fmt::println("min  = {}", select(x < y, x, y));

  fmt::println("select(x < y, x, y)    = {}", select(x < y, x, y));
  fmt::println("select(x < y, x, 2)    = {}", select(x < y, x, vec_t{s_t{2}}));
  fmt::println("select(x < y, 2, y)    = {}", select(x < y, vec_t{s_t{2}}, y));
  fmt::println("select(x < y, 3, 2)    = {}",
               select(x < y, vec_t{s_t{3}}, vec_t{s_t{2}}));
  fmt::println("select(filter01, x, y) = {}", select(filter01, x, y));
  fmt::println("select(filter02, x, 2) = {}",
               select(filter02, x, vec_t{s_t{2}}));
  fmt::println("select(filter02, 3, y) = {}",
               select(filter02, vec_t{s_t{3}}, y));
  fmt::println("sqrt(x) * sqrt(x)      = {}", sqrt(x) * sqrt(x));
  fmt::println("2.0 * sqrt(x)          = {}", vec_t{s_t{2}} * sqrt(x));
  fmt::println("sqrt(x) * 2.0          = {}", sqrt(x) * vec_t{s_t{2}});
}

void simd_examples() {
  using namespace aks::simd;
  simd_check<vec1d>(from_bools<mask<vec1d::spec>>(true),
                    from_bools<mask<vec1d::spec>>(false));
  simd_check<vec1f>(from_bools<mask<vec1f::spec>>(true),
                    from_bools<mask<vec1f::spec>>(false));
  simd_check<vec1i>(from_bools<mask<vec1i::spec>>(true),
                    from_bools<mask<vec1i::spec>>(false));
  simd_check<vec4d>(from_bools<mask<vec4d::spec>>(true, false, true, true),
                    from_bools<mask<vec4d::spec>>(false, false, true, false));
  simd_check<vec8f>(from_bools<mask<vec8f::spec>>(true, false, true, true,
                                                  false, true, true, true),
                    from_bools<mask<vec8f::spec>>(false, false, true, false,
                                                  false, false, true, false));
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

void semd_examples() {
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
}

int main() {
  simd_examples();
  semd_examples();
  return 0;
}
