# KB-backbone
KB-backbone is a ROS library that enables higher level communication between ROS nodes.

Currently, two functionalities are provided:
1. Frequency bounded publish/subscribe using a multiplexer node
2. Version checking for custom messages published on a topic

# Installation
Clone this repository in the `src` folder of your catkin workspace and run `catkin_make` from the root of the workspace.

For instance, if your catkin workspace is `/home/<user>/ws` you may run the following commands:

```bash
$ git clone https://github.com/unipi-smartapp-2021/KB-backbone.git  /home/<user>/ws/src
$ cd /home/<user>/ws  
$ catkin_make
```

# Usage

## Multiplexer

### Using rosrun
The library provides the ```multiplexer``` executable to quickly start a multiplexer on a given topic. As an example, if you want to provide timed subscriptions to a std_msgs/String topic ```/test```  at 5,10,20,and 50 Hz just run
```bash
$ rosrun backbone multiplexer --topic /test --message_type std_msgs/String --rates 5 10 20 50
```
This will create 4 new topics: **testRated5Hz**, **testRated10Hz**, **testRated20Hz** and **testRated50Hz**. You can subscribe to one of these to get messages at the indicate maximum frequency. 
Please remeber to provide **positive** and **integer** values to **--rates**. RatedTopic expects them to also be sorted in ascended order and made of distinct values, but this script does that automatically.

You can also setup multiple multiplexer of this nodes using ```roslaunch```, just remember to pass the required arguments **--topic** **--message_type** and **--rates**.
### As a library
Some example nodes can be found in the [`test_nodes`](https://github.com/unipi-smartapp-2021/KB-backbone/tree/main/test_nodes) folder, they offer a hint on how to use the classes exposed by this library.

You can try such nodes with the launch files provided in [`launch`](https://github.com/unipi-smartapp-2021/KB-backbone/tree/main/launch) by running the following commands:

- [C++ examples](test_nodes/cpp):
    
    ```bash 
    $ roslaunch backbone backbone.launch
    ```
    
- [Python examples](test_nodes/python):
    
    ```bash 
    $ roslaunch backbone python.launch
    ```

A more detailed API reference is available in [USAGE.md](./USAGE.md).

## Version Checking
The library provides the  ```version_checker``` executable. This executable needs two parameter on the parameter server to function:
1. *t_version* is an integer indicating the current version constraint that each custom message published on topic *t* must satisfy
2. *VERSION_FIELD* is a string parameter that indicates the **name** of the field that each custom message must use to store the version value.

```bash
$ rosparam set topic_version 1
```

```bash
$ rosparam set VERSION_FIELD version
```
To start the version checker run

```bash
$ rosrun backbone multiplexer --topic test --message-type backbone/TestMsg
```
**N.B**: do not put any slash at the start of the topic name, it is automatically included by the script

The ```--message-type``` flag is optional, if it is not set, the node will wait for the first message to be published on the topic to access informations about its type.

You can also configure this node in a roslaunch file. Rememeber to load the two required parameters on the parameter server.
# Docs

We use **doxygen** to generate docs from comments in the source code. Run 

```bash
$ doxygen
```

from the root of the projects to generate the docs. Docs are built inside the `docs` folder. 

You can view the html version by opening the `index.html` file created in `docs/html` with your browser of choice.
