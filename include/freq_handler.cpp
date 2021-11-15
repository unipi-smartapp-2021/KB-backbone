#pragma
#include "subscriber.hpp"
namespace sa {
namespace kb {
class FrequencyMultiplexerNode {
 public:
  FrequencyMultiplexerNode(ros::NodeHandle&& source) : source_{source} {}
  FrequencyMultiplexerNode(ros::NodeHandle const& source) : source_{source} {}

 private:
  /// This method is really swag
  auto Start() -> void {
    // ...
  }

  ///
  ros::NodeHandle source_;

  // DK, sinceramente fa cagare ros ahah
  std::vector<sa::kb::Subscriber> subs; /// Struttura per organizzare <Subscriber, Frequency>
  ///
  /// In caso un subscriber  volesse registrarsi senza usare una frequenza, ma essere informato
  /// quando un messaggio e` disponibile?
  /// std::optional<Frequency>?
};
} // namespace kb
} // namespace sa
