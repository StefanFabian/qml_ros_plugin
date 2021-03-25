====
Time
====

To preserve the accuracy and allow for compatible serialization of message objects, custom
:cpp:class:`Time <qml_ros_plugin::Time>` and :cpp:class:`WallTime <qml_ros_plugin::WallTime>` anonymous datatypes were
introduced.
This :cpp:class:`Time <qml_ros_plugin::Time>` datatype is used for time fields in received messages and can be obtained
using the :cpp:class:`Time <qml_ros_plugin::TimeSingleton>` singleton.

Example:

.. code-block:: qml

  property var currentTime: Time.now()
  property var currentWallTime: WallTime.now()

The :cpp:class:`Time <qml_ros_plugin::TimeSingleton>` singleton wraps most static methods of ``ros::Time`` whereas the
instances obtained using either :cpp:func:`Time.now() <qml_ros_plugin::TimeSingleton::now>` or
:cpp:func:`Time.create() <qml_ros_plugin::TimeSingleton::create>` contain the instance methods and properties
for ``sec`` and ``nsec``.

Both wrapper types can be converted to QML/JavaScript ``Date`` objects using the :cpp:func:`toJSDate() <qml_ros_plugin::Time::toJSDate>`
function at the cost of micro- and nanosecond accuracy.

Please note that due to limitations in QML and JavaScript mathematical operations for Time and WallTime are not possible.

API
---
.. doxygenclass:: qml_ros_plugin::Time
  :members:

.. doxygenclass:: qml_ros_plugin::WallTime
  :members:

.. doxygenclass:: qml_ros_plugin::TimeSingleton
  :members:

.. doxygenclass:: qml_ros_plugin::WallTimeSingleton
  :members:
