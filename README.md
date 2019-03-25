## QmlRosPlugin
Connects QML user interfaces to the Robot Operating System (ROS).  
Currently, has support for the following:

### Subscribers
Can be used to subscribe to any topic and message type.  
Array size is currently limited to 10000.

Usage example:
```
import RosPlugin 1.0

Item {
  width: 600
  height: 400
  
  Subscriber {
    id: subscriber
    topic: "/test"
    onNewMessage: textField.text = message.data 
  }
  
  Text {
    text: "You can use the message directly: " + subscriber.message.data
  }
  
  Text {
    id: textField
    text: "Or you can use the newMessage signal."
  }
}
```


### Roadmap
[] ImageTransport   
[] Publishers(?)  
