#pragma once

#include <chrono>
#include <condition_variable>
#include <memory>
#include <string>
#include <vector>

#include "ros/ros.h"

#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/leaf_system.h"

namespace dairlib {
namespace systems {

/**
 * Receives ROS messages from a given topic and outputs them to a
 * System<double>'s port. This class stores the most recently processed ROS
 * message in the State. When a ROS message arrives asynchronously, an update
 * event is scheduled to process the message and store it in the State at the
 * earliest possible simulation time. The output is always consistent with the
 * State.
 *
 * To process a ROS message, CalcNextUpdateTime() needs to be called first to
 * check for new messages and schedule a callback event if a new ROS message
 * has arrived. The message is then processed and stored in the Context by
 * CalcDiscreteVariableUpdates() or CalcUnrestrictedUpdate() depending on the
 * output type. When this system is evaluated by the Simulator, all these
 * operations are taken care of by the Simulator. On the other hand, the user
 * needs to manually replicate this process without the Simulator.
 *
 * (Direct clone of LcmSubscriberSystem with pared-down features.)
 */
template <typename RosMessage>
class RosSubscriberSystem : public drake::systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RosSubscriberSystem)

  /**
   * A factory method that returns an %RosSubscriberSystem that takes
   * Value<RosMessage> message objects on its sole abstract-valued input port.
   *
   * @tparam RosMessage message type to serialize.
   *
   * @param[in] topic The ROS topic to subscribe to.
   *
   * @param node_handle ROS node handle for context (to make the subscriber).
   */
  static std::unique_ptr<RosSubscriberSystem<RosMessage>> Make(
      const std::string& topic, ros::NodeHandle* node_handle) {
    return std::make_unique<RosSubscriberSystem<RosMessage>>(topic,
                                                             node_handle);
  }

  /**
   * Constructor that returns a subscriber System that provides message objects
   * on its sole abstract-valued output port.  The type of the message object is
   * determined by the @p serializer.
   *
   * @param[in] channel The ROS topic on which to subscribe.
   *
   * @param node_handle The ROS context.
   */
  RosSubscriberSystem(const std::string& topic, ros::NodeHandle* node_handle)
      : topic_(topic), node_handle_(node_handle) {
    DRAKE_DEMAND(node_handle_ != nullptr);


    subscriber_ = node_handle->subscribe(
        topic, 1, &RosSubscriberSystem<RosMessage>::HandleMessage, this);


    // Declare our two states (message_value, message_count).
    static_assert(kStateIndexMessage == 0, "");
    auto message_state_index =
        this->DeclareAbstractState(drake::Value<RosMessage>(RosMessage{}));
    static_assert(kStateIndexMessageCount == 1, "");
    this->DeclareAbstractState(drake::Value<int>(0));

    // Our sole output is the message state.
    this->DeclareStateOutputPort(drake::systems::kUseDefaultName,
        message_state_index);

    set_name(make_name(topic_));
    start_ = std::chrono::steady_clock::now();
    time_ = 0;
  }

  ~RosSubscriberSystem() override{};

  const std::string& get_topic_name() const { return topic_; }

  double message_time() const { return time_; }

  /// Returns the default name for a system that publishes @p topic.
  static std::string make_name(const std::string& topic) {
    return "RosSubscriberSystem(" + topic + ")";
  }

  /**
   * Blocks the caller until @p old_message_count is different from the
   * internal message counter, and the internal message counter is returned.
   */
  int WaitForMessage(int old_message_count) const {
    // The message buffer and counter are updated in HandleMessage(), which is
    // a callback function invoked by a potentially different thread. Thus,
    // for thread safety, these need to be properly protected by a mutex.
    std::unique_lock<std::mutex> lock(received_message_mutex_);

    // This while loop is necessary to guard for spurious wakeup:
    // https://en.wikipedia.org/wiki/Spurious_wakeup
    while (old_message_count == received_message_count_)
      // When wait returns, lock is atomically acquired. So it's thread safe to
      // read received_message_count_.
      received_message_condition_variable_.wait(lock);
    int new_message_count = received_message_count_;
    lock.unlock();

    return new_message_count;
  }

  /**
   * Returns the internal message counter. Meant to be used with
   * `WaitForMessage`.
   */
  int GetInternalMessageCount() const {
    std::unique_lock<std::mutex> lock(received_message_mutex_);
    return received_message_count_;
  }

  /**
   * Returns the message counter stored in @p context.
   */
  int GetMessageCount(const drake::systems::Context<double>& context) const {
    // Gets the last message count from abstract state.
    return context.get_abstract_state<int>(kStateIndexMessageCount);
  }

 protected:
  void DoCalcNextUpdateTime(const drake::systems::Context<double>& context,
      drake::systems::CompositeEventCollection<double>* events, double* time)
      const override {
    // We do not support events other than our own message timing events.
    LeafSystem<double>::DoCalcNextUpdateTime(context, events, time);
    DRAKE_THROW_UNLESS(events->HasEvents() == false);
    DRAKE_THROW_UNLESS(std::isinf(*time));

    // Do nothing unless we have a new message.
    const int last_message_count = GetMessageCount(context);
    const int received_message_count = GetInternalMessageCount();
    if (last_message_count == received_message_count) {
      return;
    }


    *time = context.get_time();

    drake::systems::EventCollection<
        drake::systems::UnrestrictedUpdateEvent<double>>&
        uu_events = events->get_mutable_unrestricted_update_events();
    uu_events.AddEvent(drake::systems::UnrestrictedUpdateEvent<double>(
            drake::systems::Event<double>::TriggerType::kTimed));
  }

  void DoCalcUnrestrictedUpdate(
      const drake::systems::Context<double>&, const std::vector<
          const drake::systems::UnrestrictedUpdateEvent<double>*>&,
      drake::systems::State<double>* state) const override {
    ProcessMessageAndStoreToAbstractState(&state->get_mutable_abstract_state());
  }

  void SetDefaultState(const drake::systems::Context<double>& context,
                       drake::systems::State<double>* state) const override {
    ProcessMessageAndStoreToAbstractState(&state->get_mutable_abstract_state());
  };

 private:
  void ProcessMessageAndStoreToAbstractState(
      drake::systems::AbstractValues* abstract_state) const {
    std::lock_guard<std::mutex> lock(received_message_mutex_);
    abstract_state->get_mutable_value(kStateIndexMessage)
        .get_mutable_value<RosMessage>() = received_message_;
    abstract_state->get_mutable_value(kStateIndexMessageCount)
        .get_mutable_value<int>() = received_message_count_;
  }

  // Callback entry point from ROS into this class.
  void HandleMessage(const RosMessage& message) {
    SPDLOG_TRACE(drake::log(), "Receiving ROS {} message", topic_);
    std::lock_guard<std::mutex> lock(received_message_mutex_);
    received_message_ = message;
    received_message_count_++;
    received_message_condition_variable_.notify_all();
    time_ =
        (duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - start_)).count()/1.0e6;
  }

  // The topic on which to receive ROS messages.
  const std::string topic_;

  // The mutex that guards received_message_ and received_message_count_.
  mutable std::mutex received_message_mutex_;

  // A condition variable that's signaled every time the handler is called.
  mutable std::condition_variable received_message_condition_variable_;

  mutable double time_;
  mutable std::chrono::time_point<std::chrono::steady_clock> start_;

  // The most recently received ROS message.
  RosMessage received_message_{};

  // A message counter that's incremented every time the handler is called.
  int received_message_count_{0};

  ros::NodeHandle* const node_handle_{};
  ros::Subscriber subscriber_;

  constexpr static int kStateIndexMessage = 0;
  constexpr static int kStateIndexMessageCount = 1;
};

}  // namespace systems
}  // namespace dairlib
