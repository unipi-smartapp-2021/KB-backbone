#include <sstream>
#include <tuple>
#include <vector>

#include "config.hpp"
#include "ros/message_traits.h"
#include "ros/rate.h"
#include "ros/ros.h"
#include "ros/subscriber.h"
#include "ros/subscription_callback_helper.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/String.h"
#include "subscriber.hpp"
#include "topic_tools/shape_shifter.h"

class Subscribable {
 public:
  ///
  /// Comunque, questa funzione stavo pensando che non ha senso che sia nell'interfaccia
  /// perche` gli altri nodi (es quelli che faranno filtering su messaggio) non useranno gli stessi
  /// parametri
  //                                  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~v
  /// ad esempio, prendiamo il subscribe ad un nodo che fa filtering
  /// e` ragionevole supporre che il sottoscrivente passi ad esempio una stringa con il campo che gli
  /// interessa, invece di `ros::Rate`

  /// Infatti stavo pensando ad una cosa tipo il command patter
  /// incapsulare la richiesta di sottoscrizione con un oggetto callable
  /// cosi` l'utente puo` definire la propria logica
  /// e noi dobbiamo solo fare `policy.run()`

  /// sto facendo brainstorming eh, magari so cazzate ahah sicuro 100%
  /// esatto! a quel punto l'interfaccia e` un solo metodo `RUN` per la policy
  /// e quindi portremmo avere questa classe
  virtual auto Subscribe(ros::NodeHandle const& subscriber) -> void = 0;
};

//
// e questa
template<typename T>
class CustomSubscribable {
  virtual auto SubscribeWithPolicy(ros::NodeHandle const&             subscriber,
                                   ros::SubscriptionCallbackHelperPtr policy) -> void = 0;
};

class Publishable {
 public:
  virtual auto Publish() -> void = 0;
};

template<typename T>
void SerializeToByteArray(const T& msg, std::vector<uint8_t>& destination_buffer) {
  const uint32_t length = ros::serialization::serializationLength(msg);
  destination_buffer.resize(length);
  // copy into your own buffer
  ros::serialization::OStream stream(destination_buffer.data(), length);
  ros::serialization::serialize(stream, msg);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "talker");

  ros::NodeHandle node_handler;

  topic_tools::ShapeShifter shape_shifter;
  shape_shifter.morph(ros::message_traits::MD5Sum<std_msgs::String>::value(),
                      ros::message_traits::DataType<std_msgs::String>::value(),
                      ros::message_traits::Definition<std_msgs::String>::value(),
                      "");

  ros::Publisher test_pub = shape_shifter.advertise(node_handler, sa::kTestTopic, 1);
  auto           testmsg = sensor_msgs::JointState();
  testmsg ros::Rate loop_rate(10);

  std_msgs::String msg;

  std::vector<uint8_t> buffer;

  uint32_t count = 0;
  while (ros::ok()) {
    // update the message
    auto s = std::stringstream();

    s << "Hi " << count;

    msg.data = s.str();

    // serialize the message inside the shape_shifter instance
    SerializeToByteArray(msg, buffer);
    ros::serialization::OStream stream(buffer.data(), buffer.size());
    shape_shifter.read(stream);
    test_pub.publish(shape_shifter);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  return 0;
}