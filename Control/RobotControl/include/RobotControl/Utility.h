#pragma once

#include <cmath>
#include "vec2/vec2.h"

#include <complex>
#include <array>
#include <limits>
#include <algorithm>
#include <optional>
#include <chrono>

namespace utils {

  constexpr double PI{ M_PI };

  inline constexpr double deg2rad(double deg) noexcept {
    return deg / 180.0 * PI;
  }
  inline constexpr double rad2deg(double deg) noexcept {
    return deg * 180.0 / PI;
  }

  /**
   * \brief Converts angle into [-PI, PI) range
   */
  inline double normalize_angle(double x) noexcept {
    x = fmod(x + PI, 2 * PI);
    if (x < 0) x += 2 * PI;
    return x - PI;
  }

  template <class T>
  inline constexpr T sign(T val) noexcept {
    if (val > 0) return 1;
    if (val == 0) return 0;
    return -1;
  }

  inline constexpr double sqr(double value) {
    return value * value;
  }


  /**
   * \brief Finds angle equivalent to \param to which is "closest" to \param from
   *
   */
  inline double closest_angle(double from, double to) noexcept {
    double nfrom = normalize_angle(from);
    double nto = normalize_angle(to);

    double diff = normalize_angle(nto - nfrom);

    return from + diff;
  }

  using Point = vec2;


  inline std::array<std::complex<double>, 3> solve_cubic(const std::complex<double>& a, const std::complex<double>& b,
                                                         const std::complex<double>& c, const std::complex<double>& d) {
    using cmpl = const std::complex<double>;

    cmpl delta_0 = b * b - 3.0 * a * c, delta_1 = 2.0 * std::pow(b, 3) - 9.0 * a * b * c + 27.0 * a * a * d;
    cmpl C = std::pow((delta_1 + std::sqrt(delta_1 * delta_1 - 4.0 * std::pow(delta_0, 3))) / 2.0, 1.0 / 3.0);
    cmpl eps = (-1.0 + std::sqrt(std::complex<double>(-3.0))) / 2.0;

    std::array<std::complex<double>, 3> ret{ nan(""), nan(""), nan("") };

    for (int i = 0; i < 3; ++i) {
      ret[i] = -1.0 / (3.0 * a) * (b + std::pow(eps, i) * C + delta_0 / (std::pow(eps, i) * C));
    }

    return ret;
  }

  inline std::array<double, 3> solve_cubic_real(const std::complex<double>& a, const std::complex<double>& b,
                                                const std::complex<double>& c, const std::complex<double>& d) {
    const auto res = solve_cubic(a, b, c, d);
    std::array<double, 3> ret{ nan(""), nan(""), nan("") };

    int j = 0;
    for (const auto& x : res) {
      if (std::abs(x.imag()) < 0.01) {
        ret[j++] = x.real();
      }
    }

    return ret;
  }


  inline std::array<std::complex<double>, 2> solve_quadratic(const std::complex<double>& a,
                                                             const std::complex<double>& b,
                                                             const std::complex<double>& c) {
    std::array<std::complex<double>, 2> ret;
    ret[0] = (-b + std::sqrt(b * b - 4.0 * a * c)) / (2.0 * a);
    ret[1] = (-b - std::sqrt(b * b - 4.0 * a * c)) / (2.0 * a);

    return ret;
  }


  /**
   * @brief Combines the arguments based on ration from template parameters
   * @details Example1: combiner<1, 1>(a, b) -> returns 0.5(a + b)
   * Example2: combiner<2, 1>(a, b) -> 2/3*a + 1/3*b
   * @tparam r1..r5 ratio as integer
   */
  template <int r1, int r2, int r3 = 0, int r4 = 0, int r5 = 0>
  struct combiner {
    constexpr double operator()(double arg1, double arg2, double arg3 = 0, double arg4 = 0, double arg5 = 0) const {
      return (1.0 / (r1 + r2 + r3 + r4 + r5)) * (r1 * arg1 + r2 * arg2 + r3 * arg3 + r4 * arg4 + r5 * arg5);
    }
  };

  /**
   * @brief scales @p v from range [ @p in_min, @p in_max ] to range [ @p out_min, @p out_max ]
   * @return scaled value
   */
  inline constexpr double map(double v, double in_min, double in_max, double out_min, double out_max) {
    return (v - in_min) / (in_max - in_min) * (out_max - out_min) + out_min;
  }
  /**
   * @brief scales from [ - @p in , @p in ] to [ - @p out , @p out ]
   * @see map(double, double, double, double, double)
   */
  inline constexpr double map(double v, double in, double out) {
    return map(v, -in, in, -out, out);
  }

  /**
   * @brief clamps input and output to specified limits
   * @see map(double, double, double, double, double)
   */
  inline constexpr double map_clamped(double v, double in_min, double in_max, double out_min, double out_max) {
    using std::clamp;
    const double mapped = map(clamp(v, in_min, in_max), in_min, in_max, out_min, out_max);
    return clamp(mapped, out_min, out_max);
  }
  /**
   * @brief clamps input and output to specified limits
   * @see map(double, double, double)
   */
  inline constexpr double map_clamped(double v, double in, double out) {
    return map_clamped(v, -in, in, -out, out);
  }


  /**
   * @brief Holds the last value of an std::optional input
   *
   * @tparam the type that will be returned
   */
  template <class T>
  struct OptValueHold {
  private:
    T last_val_{};

  public:
    const T& operator()(const std::optional<T>& in) {
      if (in) {
        last_val_ = *in;
      }
      return last_val_;
    }
    const T& operator()() const {
      return last_val_;
    }
  };

  using clock = std::chrono::steady_clock;

  template <class T>
  inline bool elapsed(const clock::time_point& since, const T& dur) {
    return clock::now() - since > dur;
  }

  inline bool elapsed(const clock::time_point& since, double dur) {
    return elapsed(since, std::chrono::milliseconds(static_cast<int>(1000 * dur)));
  }

  template <class T>
  inline bool pending(const clock::time_point& since, const T& dur) {
    return !elapsed(since, dur);
  }

  inline uint32_t millis() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(clock::now().time_since_epoch()).count();
  }

  inline clock::time_point now() {
    return clock::now();
  }

};  // namespace utils
