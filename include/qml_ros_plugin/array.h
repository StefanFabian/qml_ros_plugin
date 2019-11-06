// Copyright (c) 2019 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef QML_ROS_PLUGIN_ARRAY_H
#define QML_ROS_PLUGIN_ARRAY_H

#include <QObject>
#include <QVariant>

#include <ros_babel_fish/messages/array_message.h>
#include <ros_babel_fish/babel_fish.h>

namespace qml_ros_plugin
{

/*!
 * @brief View on an array field of a message.
 * This allows access on array elements with lazy copy mechanism.
 */
class Array : public QObject
{
Q_OBJECT
  // @formatter:off
  //! The length of the array, i.e., the number of elements.
  Q_PROPERTY( size_type length READ length WRITE setLength NOTIFY lengthChanged )
  // @formatter:on
public:
  using size_type = QVariantList::size_type;

  Array( ros_babel_fish::TranslatedMessage::ConstPtr translated_message,
         const ros_babel_fish::ArrayMessageBase *message ); // TODO: Check if raw array data can be passed to qml as data property

//  Array( const Array &other );

  ~Array() override = default;

  size_type length() const;

  size_type size() const { return length(); }

  void setLength( size_type value );

  /*!
   * If the index is out of the bounds of the array, an empty QVariant is returned.
   *
   * @param index Index of the retrieved element.
   * @return The array element at the given index.
   */
  Q_INVOKABLE QVariant at( size_type index ) const;

  /*!
   * Changes the array content by removing delete_count elements at index and inserting the elements in items.
   * This method can be used to remove, replace or add elements to the array.
   *
   * @warning If the operation is not limited to the end of the array, it requires a deep copy of the message array.
   *
   * @param start The index at which to start changing the array. If greater than the length of the array, start will
   *   be set to the length of the array. If negative, it will begin that many elements from the end of the array
   *   (with origin -1, meaning -n is the index of the nth last element and is therefore equivalent to the index of
   *   array.length - n). If the absolute value of start is greater than the length of the array, it will begin from
   *   index 0.
   * @param delete_count The number of elements to delete starting at index start. If delete_count is greater than
   *   the number of elements after start, all elements from start to the length of the array are removed.
   *   If delete_count is 0, no elements are removed, e.g., for a insert only operation.
   * @param items The items that will be inserted at start.
   */
  Q_INVOKABLE void spliceList( size_type start, size_type delete_count, const QVariantList &items );

  /*!
   * Adds the given value to the end of the array.
   *
   * @param value The item that is added.
   */
  Q_INVOKABLE void push( const QVariant &value );

  /*!
   * Adds the given value to the front of the array.
   *
   * @warning This requires a deep copy of the message array on first call whereas appending can be done without copying
   *   the array.
   * @param value The item that is added.
   */
  Q_INVOKABLE void unshift( const QVariant &value );

  /*!
   * Removes the last element and returns it.
   * @return The removed element or an empty QVariant if the array is empty.
   */
  Q_INVOKABLE QVariant pop();

  /*!
   * Removes the first element and returns it.
   * @warning This requires a deep copy of the message array on first call whereas appending can be done without
   *   copying the array.
   * @return The removed element or an empty QVariant if the array is empty.
   */
  Q_INVOKABLE QVariant shift();

  /* Internal functions */
  bool _isModified( size_type index ) const;

  bool _inCache() const;

  const ros_babel_fish::ArrayMessageBase *_message() const;

signals:

  void lengthChanged();

private:
  void enlargeCache( size_type size );

  void fillCache();

  ros_babel_fish::TranslatedMessage::ConstPtr translated_message_;
  const ros_babel_fish::ArrayMessageBase *message_;
  mutable QVariantList cache_;
  bool all_in_cache_;
  QList<bool> modified_;
  size_type length_;
};
} // qml_ros_plugin

#endif // QML_ROS_PLUGIN_ARRAY_H
