==========
Quickstart
==========

This library provides convenient access of ROS concepts and functionalities
in QML.

Installation
============

From Source
-----------

To install ``qml_ros_plugin`` from source, clone the
`repo <https://github.com/StefanFabian/qml_ros_plugin>`_.
Next, open a terminal and ``cd`` into the repo folder.
To install create a build folder, ``cd`` into that folder and run
``cmake ..`` followed by ``sudo make install``.

::

   mkdir build && cd build
   cmake ..
   sudo make install

Usage
=====

To use the plugin import ``Ros`` in QML.

.. code-block:: qml

  import Ros 1.0

Now, you can use the provided components such as ``Subscriber`` and
``TfTransform`` and the Ros singleton to create a ``Publisher`` or the
``Service`` singleton to call a service.

As a simple example, a ``Subscriber`` can be created as follows:

.. code-block:: qml
  :linenos:

  Subscriber {
    id: mySubscriber
    topic: "/intval"
  }

For more in-depth examples, check out the :ref:`Examples` section.

Initialization
--------------
Before a ``Subscriber`` can receive messages, a ``Publisher`` can publish
messages, etc. the node has to be initialized.
If your application calls ``ros::init`` on startup, you don't have to do
anything. However, in cases where you can't call ``ros::init`` from the C++
entry function, you can use the ``init`` function of the ``Ros`` singleton:

.. code-block:: qml
  :linenos:

  ApplicationWindow {
    /* ... */
    Component.onCompleted: {
      Ros.init("node_name");
    }
  }

Shutdown
--------
To make your application quit when ROS shuts down, e.g., because of a
``Ctrl+C`` in the console or a ``rosnode kill`` request, you can connect
to the ``Shutdown`` signal:

.. code-block:: qml
  :linenos:

  ApplicationWindow {
    Connections {
      target: Ros
      onShutdown: Qt.quit()
    }
    /* ... */
  }

For more on that, check out the :ref:`Ros Singleton`.
