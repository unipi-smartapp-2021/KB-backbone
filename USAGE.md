# C++ Usage

The main template exposed is ```RatedTopic```. Let's say for example that you want to receive messages from the topic **sensor_topic**. You know that this topic gets published at 100 hz, but you have clients that wants to receive messages at 20 or 10 hz. You must follow these steps:
## Server side 

1.  initialize your node (**IMPORTANT**, no init call is made from inside the library)
    ```c++
    ros::init(argc, "my node")
    ```
2. Initialize the multiplexer with the target rates. **The list of target must be composed of distinct positive values and sorted or the creation will fail**.
    ```c++
      auto rates = std::vector<unsigned>({10, 20}); //setup list of target rates. 
      // initialize the multiplexer
      // The type of the template must be the c++ type of the messages published on the topic
      auto multiplexer = sa::kb::RatedTopic<std_msgs::String>("sensor_topic",rates)
      // start the multiplexer node
      multiplexer.Run()
    
    ```
## Client side

1. Setup node for calling the RateTopic service
    ```c++
        auto client = handle.serviceClient<backbone::RateTopic>("sensor_topicRated");
        auto service = backbone::RateTopic();
    ```
2. Initialize your request with the rate that you prefer.
    ```c++
        service.request.rate = 10;
    ```
3. Call the service 
     ```c++
        if (!client.call(service)) {
        	ROS_ERROR("Failed to call service");
        	return -1;
        }
    ```
4. Subscribe to the topic returned in the response with your callback
     ```c++
     	auto subscriber = handle.subscribe(service.response.topic, 1, Callback);
    ```

# Python usage

The python setup is very similar  but there are some differences:

1. **Server side**: since there are no templates in python, the message type must be passed as a constructor parameter. 

   ```python
   	rospy.init_node("freq_multiplexer") # ALWAYS EXECUTE THIS BEFORE CREATING THE MULTIPLEXER
   
       m = backbone.RatedTopic("sensor_topic", std_msgs.msg.String, rates=[10, 20])
       m.run()
   ```

2.  **Client side**: the service call returns a response object and not a boolean 

   ```python
   	rospy.wait_for_service("sensor_topicRated")
       try:
           throttletopic = rospy.ServiceProxy("sensor_topicRated", RateTopic)
           resp: RateTopicResponse = throttletopic(5) 	
           topic = resp.topic
           rospy.Subscriber(topic, data_class=std_msgs.msg.String, callback=callback)
           rospy.loginfo(f"listening on topic {topic}")
           rospy.spin()
   	except rospy.ServiceException as e:
           print(f"Failed service call: {e}")
      	return 0
   ```

   

# Various 

1. ```RatedPublisher```

   The multiplexer uses internally the ```RatedPublisher``` class. This is also exposed and can be used to throttle a publisher.

   - C++

   ```c++
   auto handle = ros::NodeHandle();
   auto normal_publisher = handle.advertise<std_msgs::String>("sensor_topic", 10);
   auto throttled_publisher = sa::kb::RatedPublisher(normal_publisher, 10);
   auto msg = std_msgs::String();
   msg.data = "message";
   while (ros::ok()){
       throttled_publisher.publish(msg) // will actually publish only at a frequency of 10Hz
   }
   
   ```

   - Python 

    ```python
     normal_publisher = rospy.Publisher("sensor_topic", data_class=std_msgs.msg.String,queue_size=10)
     throttled_publisher = backbone.RatedPublisher(normal_publisher,10)
     msg = std_msgs.msg.String()
     msg.data = "message"
     while not rospy.is_shutdown():
         throttled_publisher.publish(msg) # will actually publish only at a frequency of 10Hz
    ```

     

 2. The ```RatedTopic``` multiplexer can  keep track of the average delay that occurs between receiving a message from the original topic and forwarding it to the actual clients. This can be useful for debugging purposes, for example to see if too much time is spent in the actual throttling. To enable this you must subscribe to the topic _<throttled_topic>RatedDelay_ and you will start receiving the average delay in the _Duration_ message format.

    - C++
    
    ```c++
    #include <ros/init.h>
    #include <ros/rate.h>
    #include <ros/ros.h>
    #include <std_msgs/Duration.h>
    
    #include <cstdlib>
    
    auto Callback(boost::shared_ptr<std_msgs::Duration const> const& message) -> void {
      ROS_INFO("Debugger) Delay is \"%ld\" microseconds", message->data.toNSec() / 1000);
    }
    
    auto main(int argc, char* argv[]) -> int {
      ros::init(argc, argv, "Debugger");
    
      auto handle = ros::NodeHandle();
      auto client = handle.subscribe(std::string("sensor_topicRatedDelay", 1, Callback);
      ros::spin();
    
      return 0;
    }
    ```
    
    - Python
    
    ```python
    import rospy
    from std_msgs.msg import Duration
    
    def debug(msg: Duration) -> None:
        rospy.loginfo(f"(DEBUGGER) Delay is :{msg.data.to_nsec() / 1000} microseconds")
    
    def main() -> int:
        rospy.init_node("debugger")
        rospy.Subscriber("sensor_topicRatedDelay",Duration,callback=debug, queue_size=1)
        rospy.spin()
    
    if __name__ == "__main__":
        raise SystemExit(main())
    ```
    
    
    
    