# KB-backbone
ROS library that enables frequency bounded communication between ROS nodes

# Installation
Clone this repository in the **src** folder of your catkin workspace and then run **catkin_make** from the root of the workspace.

If, for example, your catkin workspace is /home/<user>/ws run these commands:

```bash
$ git clone https://github.com/unipi-smartapp-2021/KB-backbone.git  /home/<user>/ws/src
$ cd /home/<user>/ws  
$ catkin_make
```



# Usage
In the **test_nodes** folder there are some example nodes to show how to use the classes exposed by the library.
You can run these nodes with the launch files provided in **launch** with the following commands:

- [C++ examples](test_nodes/cpp):
    
    ```bash 
    $ roslaunch backbone backbone.launch
    ```
    
- [Python examples](test_nodes/python):
    
    ```bash 
    $ roslaunch backbone python.launch
    ```

A more detailed explanation of the APIs is in [USAGE.md](./USAGE.md)

# Docs

We use **doxygen** to generate docs from comments in our code. Run 

```bash
$ doxygen
```

from the root of the projects to generate the docs. These are built inside the **docs** folder.  You can visit the html version by opening the index.html file created in docs/html with any browser.
