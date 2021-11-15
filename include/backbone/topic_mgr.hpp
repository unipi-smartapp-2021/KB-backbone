// GUARDS
namespace sa {
namespace kb {
class TopicManager {
  virtual auto register_pub() -> void = 0;

  virtual auto register_sub() -> void = 0;

  virtual auto publish() -> void = 0;
};
} // namespace kb
} // namespace sa