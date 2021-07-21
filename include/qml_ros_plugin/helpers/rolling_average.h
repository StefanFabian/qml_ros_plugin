//
// Created by stefan on 21.07.21.
//

#ifndef QML_ROS_PLUGIN_ROLLING_AVERAGE_H
#define QML_ROS_PLUGIN_ROLLING_AVERAGE_H

#include <array>

namespace qml_ros_plugin
{

template<typename T, int COUNT>
class RollingAverage
{
public:
  RollingAverage()
  {
    values_.fill( 0 );
  }

  void add( const T &value )
  {
    if (count_values_ == COUNT) sum_ -= values_[index_];
    values_[index_] = value;
    sum_ += value;
    if ( ++index_ == COUNT ) index_ = 0;
    if (count_values_ != COUNT) ++count_values_;
  }

  T value() const { return count_values_ == 0 ? 0 : sum_ / count_values_; }

  operator T() const { return value(); }

private:
  std::array<T, COUNT> values_;
  T sum_ = 0;
  size_t count_values_ = 0;
  size_t index_ = 0;
};
}

#endif //QML_ROS_PLUGIN_ROLLING_AVERAGE_H
