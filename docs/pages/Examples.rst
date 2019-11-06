========
Examples
========

You can find the described example QML files in the
`qml_ros_plugin repo examples directory <https://github.com/StefanFabian/qml_ros_plugin/tree/master/examples>`_.

Subscriber
==========

The subscriber example demonstrates how to create a ``Subscriber`` in QML
using the *QML ROS Plugin*.

You can run the example using the ``qmlscene`` executable:

.. code-block::

  qmlscene subscriber.qml

Publisher
=========

The publisher example publishes an ``std_msgs/Int32`` message on the topic that
the subscriber example subscribes to.
Coincidentally, the two examples can very well be used together.

To run, run:

.. code-block::

  qmlscene publisher.qml

URDF Tutorial UI
================

This example combines several of the functionalities provided by this library
and presents a user interface for the ``urdf_sim_tutorial`` diff drive example.

First, launch the example:

.. code-block::

  roslaunch urdf_sim_tutorial 13-diffdrive.launch

Next, launch the example UI:

.. code-block::

  qmlscene urdf_tutorial_combined.qml

It provides a top down view on the position of the robot and sliders to control
the forward and angular movement.
