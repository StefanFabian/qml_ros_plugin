//
// Created by stefan on 21.07.21.
//

#ifndef QML_ROS_PLUGIN_ROLLING_AVERAGE_H
#define QML_ROS_PLUGIN_ROLLING_AVERAGE_H

#include <algorithm>
#include <array>
#include <numeric>

namespace qml_ros_plugin
{

template<typename T, int COUNT>
class RollingAverage
{
public:
  RollingAverage() { values_.fill( 0 ); }

  void add( const T &value )
  {
    if ( count_values_ == COUNT )
      sum_ -= values_[index_];
    else
      ++count_values_;
    values_[index_] = value;
    sum_ += value;
    if ( ++index_ == COUNT ) {
      index_ = 0;
      if ( std::is_floating_point<T>::value ) {
        // For floating points we calculate the whole sum once every COUNT adds
        // since the operations may not be exact.
        sum_ = std::accumulate( values_.begin(), values_.end(), 0 );
      }
    }
  }

  T value() const { return count_values_ == 0 ? 0 : sum_ / count_values_; }

  operator T() const { return value(); }

private:
  std::array<T, COUNT> values_;
  typename std::conditional<std::is_integral<T>::value, long long, double>::type sum_ = 0;
  size_t count_values_ = 0;
  size_t index_ = 0;
};
} // namespace qml_ros_plugin

#endif // QML_ROS_PLUGIN_ROLLING_AVERAGE_H
