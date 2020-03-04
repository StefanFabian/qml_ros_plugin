=============
Ros Singleton
=============

The ``Ros`` singleton provides interfaces to static methods and convenience
methods.

In QML it is available as ``Ros``, e.g.:

.. code-block:: qml

  if (Ros.ok()) console.log("Ros is ok!")

ROS Initialization
------------------

If you can not or for whatever reason do not want to initialize ROS in your
C++ entry, you can also do it in QML, e.g., in the ``onCompleted`` handler:

.. code-block:: qml

  Component.onCompleted: {
    Ros.init("node_name")
  }

As described in the API documentation for `init`_, you can pass either just the
node name or additionally use provided command line args instead of the command
line args provided to your executable. In both cases, you can also pass the
following ``RosInitOptions`` options:

* | ``NoSigintHandler``:
  | Don't install a SIGINT (e.g., ``Ctrl+C`` on terminal) handler.
* | ``AnonymousName``:
  | Anonymize the node name. Adds a random number to the node's name to make it unique.
* | ``NoRosout``:
  | Don't broadcast rosconsole output to the /rosout topic. See :ref:`Logging`

.. code-block:: qml

  Component.onCompleted: {
    Ros.init("node_name", RosInitOptions.AnonymousName | RosInitOptions.NoRosout)
  }

API
---
.. doxygenclass:: qml_ros_plugin::RosQmlSingletonWrapper
  :members:
