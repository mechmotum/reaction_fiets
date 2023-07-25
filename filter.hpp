#pragma once

namespace numbers {

template <class T>
constexpr auto pi() -> T
{
  return T(3.1415926535897932384626433);
}

template <class T>
constexpr auto sqrt2() -> T
{
  return T(1.41421356237309504880168872420969807856967187537694807317667973799);
}

}

namespace complementary_filter {

namespace detail {
template <class T>
constexpr auto squared(T x) -> T { return x * x; };
}

template <class Config>
struct lowpass
{
  using value_type = typename Config::value_type;

  struct signal
  {
    value_type value;
    value_type derivative;
  };

  value_type prev_unfiltered{};
  signal prev_filtered{{}, {}};

  static constexpr auto w0 = 2 * numbers::pi<value_type>() * Config::cutoff_frequency();
  static constexpr auto dt = Config::sample_period();

  static constexpr auto a = detail::squared(w0);
  static constexpr auto b = numbers::sqrt2<value_type>() * w0;

  // Integrate the filter state equation using the midpoint Euler method with
  // time step h
  static constexpr auto h = dt;
  static constexpr auto h2 = detail::squared(h);
  static constexpr auto denom = 4 + 2 * h * b + h2 * a;

  static constexpr auto A = (4 + 2 * h * b - h2 * a) / denom;
  static constexpr auto B = 4 * h / denom;
  static constexpr auto C = -4 * h * a / denom;
  static constexpr auto D = (4 - 2 * h * b - h2 * a) / denom;
  static constexpr auto E = 2 * h2 * a / denom;
  static constexpr auto F = 4 * h * a / denom;

  auto operator()(value_type value) -> signal
  {
    const auto x0 = value;
    const auto x1 = prev_unfiltered;
    const auto y1 = prev_filtered.value;
    const auto yd1 = prev_filtered.derivative;

    const auto y0 = A * y1 + B * yd1 + E * (x0 + x1) / 2;
    const auto yd0 = C * y1 + D * yd1 + F * (x0 + x1) / 2;

    prev_unfiltered = x0;
    prev_filtered = {y0, yd0};
    return prev_filtered;
  }
};

template <class Config>
struct highpass
{
  using value_type = typename Config::value_type;

  static constexpr auto w0 = 2 * numbers::pi<value_type>() * Config::cutoff_frequency();
  static constexpr auto dt = Config::sample_period();

  struct state_type
  {
      value_type s0;
      value_type s1;
  };

  struct return_type
  {
    value_type value;
    state_type state;
  };

  value_type prev_unfiltered{};
  state_type prev_state{{},{}};

  static constexpr auto h = dt;
  static constexpr auto a0 = numbers::sqrt2<value_type>() * h * w0;
  static constexpr auto a1 = detail::squared(h);
  static constexpr auto a2 = detail::squared(w0);
  static constexpr auto a3 = a1 * a2;
  static constexpr auto a4 = 2 * a0;
  static constexpr auto a5 = a3 + a4 + 4;
  static constexpr auto a6 = 1 / a5;
  static constexpr auto a8 = a2 * h;

  auto operator()(value_type value) -> return_type
  {

    const auto xi = value;
    const auto xim1 = prev_unfiltered;
    const auto z1im1 = prev_state.s0;
    const auto z1im2 = prev_state.s1;

    const auto a7 =
        a1 * xi + a1 * xim1 - a3 * z1im2 + a4 * z1im2 + 4 * h * z1im1 +
        4 * z1im2;

    const auto z1i =
        a6 *
        (a5 * (-a0 * z1im1 - a8 * z1im2 + h * xi + h * xim1 + 2 * z1im1) -
         a7 * a8) /
        (a0 + 2);
    const auto z2i = a6 * a7;
    const auto yi = (z1i - z1im1) / h;

    prev_unfiltered = xi;
    prev_state = {z1i, z2i};

    return return_type{yi, prev_state};
  }
};

}
