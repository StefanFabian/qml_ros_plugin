===========
Subscribers
===========

A subscriber listens for messages on a given topic.

Simple example
--------------
First, let's start with a simple example:

.. code-block:: qml
  :linenos:

  Subscriber {
    id: mySubscriber
    topic: "/intval"
  }

This creates a subscriber object that is now available and subscribed
to ``/intval``.
Let's assume the topic publishes an ``std_msgs/Int32`` message.

The ``std_msgs/Int32`` message is defined as follows:

.. code-block::

  int32 data

We can display the published value using a text field:

.. code-block:: qml
  :linenos:

  Text {
    text: "Published value was: " + mySubscriber.message.data
  }

Whenever a new message is received on ``/intval`` the message property
is updated and the change is propagated to the text field. Thus, the text
field will always display the last received value.

Full example
------------
In most cases, the above Subscriber is sufficient. However, the Subscriber
has more properties to give you more fine-grained control.

.. code-block:: qml
  :linenos:

  Subscriber {
    id: mySubscriber
    ns: "~" // Namespace
    topic: "/intval"
    queueSize: 10
    running: true
    onNewMessage: doStuff(message)
  }

The namespace ``ns`` property enables you to set the namespace of
the ``ros::NodeHandle`` that is created to subscribe to the given topic.

The ``queueSize`` property controls how many incoming messages are queued for
processing before the oldest are dropped.

Using the ``running`` property, the subscriber can be enabled and disabled.
If the property is set to ``false``, the subscriber is shut down until it is
set to ``true`` again and subscribes to the topic again.
For example, the state of a Subscriber can be toggled using a button:

.. code-block:: qml
  :linenos:

  Button {
    id: myButton
    state: "active"
    onClicked: {
      mySubscriber.running = !mySubscriber.running
      state = state == "active" ? "paused" : "active"
    }
    states: [
      State {
        name: "active"
        PropertyChanges {
          target: myButton
          text: "Unsubscribe"
        }
      },
      State {
        name: "paused"
        PropertyChanges {
          target: myButton
          text: "Subscribe"
        }
      }
    ]
  }

Whenever a new message is received, the newMessage signal is emitted and the
message is passed and can be accessed as ``message`` which technically refers
to the received message and not the message property of the Subscriber.
Untechnically, they are usually the same, though.

Finally, there's also the messageType property which holds the type of the last
received message, e.g., ``std_msgs/Int32``.

API
---

.. doxygenclass:: qml_ros_plugin::Subscriber
   :members:
