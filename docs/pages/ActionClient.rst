============
ActionClient
============

An action client can be used to send goals to an ActionServer.
One ActionClient has to have a single type of action but can send multiple goals simultaneously.
Process can be tracked using either a goal handle manually or using callbacks.

An ActionClient can be created using the :ref:`Ros Singleton` as follows:

.. code-block:: qml

  Item {
    // ...
    property var fibonacciClient: Ros.createActionClient("actionlib_tutorials/FibonacciAction", "fibonacci")
    // ...
  }

In this example an action client is created using the ``actionlib_tutorials/FibonacciAction`` action
(it is important to use the complete action type here, not any part like the ActionGoal) using the name
fibonacci on which an ActionServer should be registered (You can use the actionlib_tutorials fibonacci_server).

To send a goal, you can use the sendGoal function:

.. code-block:: qml

  goal_handle = fibonacciClient.sendGoal({ order: numberInput.value }, function (goal_handle) {
    // Do something when the goal transitions, e.g., update the status
    if (goal_handle.commState == ActionCommStates.DONE) {
      // Goal is terminated, we can access the terminalState
      switch (handle.terminalState.state) {
        case ActionTerminalStates.RECALLED:
          // Handle recalled
          break
        case ActionTerminalStates.REJECTED:
          // Handle rejected
          break
        case ActionTerminalStates.PREEMPTED:
          // Handle preempted
          break
        case ActionTerminalStates.ABORTED:
          // Handle aborted
          break
        case ActionTerminalStates.SUCCEEDED:
          // Handle succeeded
          break
        case ActionTerminalStates.LOST:
          // Handle lost
          break
      }
    }
  }, function (goal_handle, feedback) {
    // Feedback callback. Called whenever feedback is received from the action server
  })

The ``sendGoal`` function takes 3 parameters: the goal, a transition callback and a feedback callback.
Both callbacks are optional. It returns a GoalHandle which can be used query to state of the goal or
to cancel the goal. The goal_handle passed to the callbacks and the one returned are the same.

API
---

.. doxygenclass:: qml_ros_plugin::ActionClient
   :members:

.. doxygenclass:: qml_ros_plugin::GoalHandle
   :members:

.. doxygenclass:: qml_ros_plugin::TerminalState
   :members:
