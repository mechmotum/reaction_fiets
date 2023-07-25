namespace numbers {

template <class T>
constexpr auto pi() -> T{
    return T(3.1415926535897932384626433);
}

template <class T>
constexpr auto sqrt2() -> T{
    return T(1.41421356237309504880168872420969807856967187537694807317667973799 );
}

}

namespace complementary_filter {

namespace detail {
template <class T>
constexpr auto squared(T x) -> T { return x * x; };
}

template <
    class T,
    class Config
    //const T cutoff_frequency,
    //const T sample_period
>
struct lowpass
{
  using value_type = T;

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

}
