========
Services
========

Currently, there is no service client or service server implementation (It's on
my todo-list, though).
However, there is a ``Service`` singleton that can be used to call
a service.

Here's a short modified example of the service example provided in the 
:doc:`Examples`.

.. code-block:: qml

  Button {
    onClicked: {
      var result = Service.call("/add_two_ints", "roscpp_tutorials/TwoInts",
                                { a: inputA.value, b: inputB.value })
      textResult.text = !!result ? ("Result: " + result.sum) : "Failed"
    }
  }

The first argument is the ``service`` that is called, the second is the ``type``
of the service, and the final argument is the ``request`` that is sent.

The service either returns ``false`` if the call failed, ``true`` if the call
was successful but the service description has an empty return message, and the
return message of the service otherwise.

A service call is blocking and it is usually not a good idea to make blocking work
on the UI thread. For that reason, there is also the ``callAsync`` method which
runs the service call on a new separate thread and is additionally passed an
optional callback which is called after the service call completed.

.. code-block:: qml

  Button {
    onClicked: {
      textResult.text = "Loading..."
      Service.callAsync(
        "/add_two_ints", "roscpp_tutorials/TwoInts",
        { a: inputA.value, b: inputB.value },
        function (result) {
          textResult.text = !!result ? ("Result: " + result.sum) : "Failed"
        })
    }
  }

API
---

.. doxygenclass:: qml_ros_plugin::Service
  :members: