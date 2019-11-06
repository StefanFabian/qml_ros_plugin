======
Arrays
======
Due to the lazy copy mechanism, arrays differ from the standard access above.
Because the array is not copied into a QML compatible array container,
access happens with methods.

**Example**: Instead of ``path.to.array[1].someproperty``, you would write
``path.to.array.at(1).someproperty``.


API
---
.. doxygenclass:: qml_ros_plugin::Array
  :members:
