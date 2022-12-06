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

You can also conditionally initialize by checking if it was already initialized using ``Ros.isInitialized``.
As described in the API documentation for :cpp:func:`Ros.init <qml_ros_plugin::RosQmlSingletonWrapper::init>`, you can pass either just the
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

Wait For Message
----------------

If you only require a single message from a topic, you can use the ``Ros.waitForMessageAsync`` method which asynchronously
waits for a message on the given topic with an optional maximum duration to wait and once a message is received or the
maximum wait duration elapsed, the callback method is called with the received message or null if the waiting timeouted.

.. code-block:: qml

  Ros.waitForMessageAsync("/some_topic", 10000 /* wait for maximum 10 seconds */, function (msg) {
    if (!msg) {
      // Handle timeout
      return
    }
    // Handle message
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

Create Empty Message
--------------------
You can also create empty messages and service requests as javascript objects using the ``Ros`` singleton.

.. code-block:: qml

  var message = Ros.createEmptyMessage("geometry_msgs/Point")
  // This creates an empty instance of the mssage, we can override the fields
  message.x = 1; message.y = 2; message.z = 1
  // However, note that we do not call custom message constructors, hence, if the message has different default values
  // they will not be set here. This is a rarely known feature and not used often in ROS 1, though.

  // Same can be done with service requests
  var serviceRequest = Ros.createEmptyServiceRequest("std_srvs/SetBool")
  // This creates an empty instance of the service request with all members set to their default, we can override the fields
  serviceRequest.data = true

Package API
-----------
The package property provides a wrapper for ``ros::package``.

.. code-block:: qml

  // Retrieve a list of all packages
  var packages = Ros.package.getAll()
  // Get the fully-qualified path to a specific package
  var path = Ros.package.getPath("some_pkg")
  // Get plugins for a package as a map [package_name -> [values]]
  var plugins = Ros.package.getPlugins("rviz", "plugin")

Console
-------
The Ros singleton also provides access to the ``Ros`` logging functionality.
See `Logging`:ref:.

IO
--
You can also save and read data that can be serialized in the yaml format using:

.. code-block:: qml

  var obj = {"key": [1, 2, 3], "other": "value"}
  if (!Ros.io.writeYaml("/home/user/file.yaml", obj))
    Ros.error("Could not write file!")
  // and read it back
  obj = Ros.io.readYaml("/home/user/file.yaml")
  if (!obj) Ros.error("Failed to load file!")

API
---
.. doxygenclass:: qml_ros_plugin::Package
  :members:

.. doxygenclass:: qml_ros_plugin::TopicInfo
  :members:

.. doxygenclass:: qml_ros_plugin::IO
  :members:

.. doxygenclass:: qml_ros_plugin::RosQmlSingletonWrapper
  :members:
