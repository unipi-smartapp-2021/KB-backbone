# KB-backbone
KB-backbone is a ROS library that enables frequency bounded communication between ROS nodes.

# Installation
Clone this repository in the `src` folder of your catkin workspace and run `catkin_make` from the root of the workspace.

For instance, if your catkin workspace is `/home/<user>/ws` you may run the following commands:

```bash
$ git clone https://github.com/unipi-smartapp-2021/KB-backbone.git  /home/<user>/ws/src
$ cd /home/<user>/ws  
$ catkin_make
```

# Usage
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

# Docs

We use **doxygen** to generate docs from comments in the source code. Run 

```bash
$ doxygen
```

from the root of the projects to generate the docs. Docs are built inside the `docs` folder. 

You can view the html version by opening the `index.html` file created in `docs/html` with your browser of choice.
