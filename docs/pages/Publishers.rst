==========
Publishers
==========

A Publisher is used to publish messages on a given topic for delivery
to subscribers.

Simple Example
--------------
Contrary to Subscribers, a Publisher can not be instantiated but can be
created using a factory method of the Ros singleton.

.. code-block:: qml
  :linenos:

  /* ... */
  ApplicationWindow {
    property var intPublisher: Ros.advertise("std_msgs/Int32", "/intval", 10, false)
    /* ... */
  }

In order, the arguments are the ``type``, the ``topic``, the ``queueSize`` and whether
or not the topic ``isLatched``.

To publish a message using our Publisher, we can simply use the
``intPublisher`` variable.

.. code-block:: qml
  :linenos:

  SpinBox {
    id: numberInput
  }

  Button {
    onClicked: {
      intPublisher.publish({ data: numberInput.value })
    }
  }

where we pass an object with a data field containing the (integer) number of the ``SpinBox``.
This is according to the ``std_msgs/Int32`` `message definition <http://docs.ros.org/melodic/api/std_msgs/html/msg/Int32.html>`_.

API
---

Publisher
=========
.. doxygenclass:: qml_ros_plugin::Publisher
  :members:

SingleSubscriberPublisher
=========================
.. doxygenclass:: qml_ros_plugin::SingleSubscriberPublisher
  :members:
