## Warning: Currently in Alpha
While this plugin contains a significant amount of tests (~95% line coverage), it is still in alpha stadium and bugs
are expected. Be aware of this. I would not recommend using it in production systems without further testing.

## QmlRosPlugin
Connects QML user interfaces to the Robot Operating System (ROS).  
Please be aware that this loses some of the semantic information that the type of a message would normally provide.

Currently, has support for the following:

### Subscribers
Can be used to subscribe to any topic and message type.  

Usage example:
```
import Ros 1.0

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


### Tf Lookup
#### TfTransformListener
A singleton class that can be used to look up tf transforms.   
Usage example:
```
import Ros 1.0

Item {
  // ...
  Connections {
    target: TfTransformListener
    onTransformChanged: {
      var message = TfTransformListener.lookUpTransform("base_link", "world");
      if (!message.valid) {
        // Check message.exception and message.message for more info if it is available.
        return;
      }
      var translation = message.transform.translation;
      var orientation = message.transform.rotation;
      // DO something with the information
   }
  }
}
```
**Explanation**:   
You can use the TfTransformListener.lookUpTransform (and canTransform) methods anywhere in your QML code.
However, they only do this look up once and return the result. If you want to continuously monitor the transform, you
have to either connect to the *transformChanged* signal or use the convenience component TfTransform.
The message structure is identical to the ROS message, except for an added *valid* field (`message.valid`) indicating if
the transform returned is valid or not. If it is not valid, there may be a field *exception* containing the name of the
exception that occured and a field *message* with the message of the exception.

#### TfTransform
A convenience component that watches a transform.
Usage example:
```
import Ros 1.0

Item {
  // ...
  TfTransform {
    id: tfTransform
    sourceFrame: "base_link"
    targetFrame: "world"
  }
  
  Text {
    width: parent.width
    // The translation and rotation can either be accessed using the message field as in the lookUpTransform case or,
    // alternatively, using the convenience properties translation and rotation which resolve to the message fields.
    // In either case, the message.valid field should NOT be ignored.
    text: "- Position: " + tfTransform.message.transform.translation.x + ", " + tfTransform.translation.y + ", " + tfTransform.translation.z + "\n" +
          "- Orientation: " + tfTransform.message.transform.rotation.w + ", " + tfTransform.rotation.x + ", " + tfTransform.rotation.y + ", " + tfTransform.rotation.z + "\n" +
          "- Valid: " + tfTransform.message.valid + "\n" +
          "- Exception: " + tfTransform.message.exception + "\n" +
          "- Message: " + tfTransform.message.message
    wrapMode: Text.WordWrap
  }
}
```
**Explanation**:  
This component can be used to watch a transform. Whenever the transform changes, the message and the properties of the
TfTransform change and the changes are propagated by QML.

### Known Limitations
The QDateTime object used to map ros::Time does not support microsecond accuracy, hence, information about microseconds and nanoseconds is lost.

### Installation
Clone this repo into your workspace (or somewhere else haven't checked if that works, too).
```
cd {REPO}
mkdir build
cd build
cmake ..
sudo make install
```

#### Dependencies
* [ros_babel_fish](https://github.com/StefanFabian/ros_babel_fish)

### Documentation
You can find the documentation on [readthedocs.io](https://qml-ros-plugin.readthedocs.io/en/latest/index.html) or build it yourself.
#### Dependencies
* Doxygen
* Sphinx
* sphinx_rtd_theme
* Breathe

**Example for Ubuntu**  
Install dependencies
```
sudo apt install doxygen
pip3 install sphinx sphinx_rtd_theme breathe
```
#### Build documentation
```
cd REPO/docs
make html
```


### Roadmap

- [x] ~~Subscribers~~
- [x] ~~Publishers~~  
- [x] ~~Look up of Tf transforms~~   
- [ ] Transport Hints
- [ ] Callback Queues
- [ ] ImageTransport   
- [ ] Make available as ros package
- [ ] Sending of Tf transforms(?)  
