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
  | Don't broadcast rosconsole output to the /rosout topic. See :doc:`Logging`

.. code-block:: qml

  Component.onCompleted: {
    Ros.init("node_name", RosInitOptions.AnonymousName | RosInitOptions.NoRosout)
  }

Query Topics
------------

You can also use the Ros singleton to query the available topics.
Currently, three methods are provided:

* | ``QStringList queryTopics( const QString &datatype = QString())``
  | Queries a list of topics with the given datatype or all topics if no type provided.
* | ``QList<TopicInfo> queryTopicInfo()``
  | Retrieves a list of all advertised topics including their datatype. See :cpp:class:`TopicInfo`
* | ``QString queryTopicType( const QString &name )``
  | Reterieves the datatype for a given topic.

Example:

.. code-block:: qml

  // Retrieves a list of topics with the type sensor_msgs/Image
  var topics = Ros.queryTopics("sensor_msgs/Image")
  // Another slower and less clean method of this would be
  var cameraTopics = []
  var topics = Ros.queryTopicInfo()
  for (var i = 0; i < topics.length; ++i) {
    if (topics[i].datatype == "sensor_msgs/Image") cameraTopics.push(topics[i].name)
  }
  // The type of a specific topic can be retrieved as follows
  var datatype = Ros.queryTopicType("/topic/that/i/care/about")
  // Using this we can make an even worse implementation of the same functionality
  var cameraTopics = []
  var topics = Ros.queryTopics() // Gets all topics
  for (var i = 0; i < topics.length; ++i) {
    if (Ros.queryTopicType(topics[i]) == "sensor_msgs/Image") cameraTopics.push(topics[i])
  }

API
---
.. doxygenclass:: qml_ros_plugin::TopicInfo
  :members:

.. doxygenclass:: qml_ros_plugin::RosQmlSingletonWrapper
  :members:
