#pragma once
#include <cmath>
#include <type_traits>

class vec2 {
public:
  double x_ = 0, y_ = 0;

  using c_ref = const vec2&;

  vec2() = default;
  vec2(const vec2& other) = default;
  vec2(double x, double y) : x_{ x }, y_{ y } {};
  vec2& operator=(const vec2& other) {
    x_ = other.x_;
    y_ = other.y_;
    return *this;
  }
  ~vec2() = default;

  static vec2 from_angle(double x) {
    return vec2(std::cos(x), std::sin(x));
  }

  double dist(const vec2& other) const {
    return std::hypot(x_ - other.x_, y_ - other.y_);
  }
  double angle_to(const vec2& other) const {
    return std::atan2(other.y_ - y_, other.x_ - x_);
  }

  vec2& operator+=(const vec2& other) {
    x_ += other.x_;
    y_ += other.y_;
    return *this;
  }
  vec2& operator-=(const vec2& other) {
    x_ -= other.x_;
    y_ -= other.y_;
    return *this;
  }
  vec2& operator*=(const vec2& other) {
    x_ *= other.x_;
    y_ *= other.y_;
    return *this;
  }
  vec2& operator/=(const vec2& other) {
    x_ /= other.x_;
    y_ /= other.y_;
    return *this;
  }

  vec2 operator+(const vec2& other) const {
    vec2 cpy(*this);
    cpy += other;
    return cpy;
  }
  vec2 operator-(const vec2& other) const {
    vec2 cpy(*this);
    cpy -= other;
    return cpy;
  }
  vec2 operator*(const vec2& other) const {
    vec2 cpy(*this);
    cpy *= other;
    return cpy;
  }
  vec2 operator/(const vec2& other) const {
    vec2 cpy(*this);
    cpy /= other;
    return cpy;
  }

  vec2 operator-() const {
    vec2 cpy(*this);
    cpy.x_ *= -1;
    cpy.y_ *= -1;
    return cpy;
  }

  bool operator==(const vec2& other) const {
    return x_ == other.x_ && y_ == other.y_;
  }
  bool operator!=(const vec2& other) const {
    return !(*this == other);
  }
  bool equals(const vec2& other, const double err = 0.001) const {
    return dist(other) < err;
  }
};

template <typename T>
using __is_arithmetic = typename std::enable_if<std::is_arithmetic<T>::value, bool>::type;

template <class T, typename = __is_arithmetic<T>>
inline vec2 operator*(const vec2& lhs, T rhs) {
  vec2 cpy(lhs);
  cpy.x_ *= rhs;
  cpy.y_ *= rhs;
  return cpy;
}
template <class T, typename = __is_arithmetic<T>>
inline vec2 operator/(const vec2& lhs, T rhs) {
  vec2 cpy(lhs);
  cpy.x_ /= rhs;
  cpy.y_ /= rhs;
  return cpy;
}

template <class T, typename = __is_arithmetic<T>>
inline vec2 operator*(T lhs, const vec2& rhs) {
  return rhs * lhs;
}
