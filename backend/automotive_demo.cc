#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <mutex>
#include <queue>
#include <string>
#include <thread>

#include <ignition/common/Console.hh>
#include <ignition/msgs.hh>
#include <ignition/transport/Node.hh>

#include "drake/automotive/automotive_simulator.h"
#include "drake/common/find_resource.h"

using namespace drake;
using namespace automotive;

namespace delphyne {
namespace backend {

/// \brief Flag used to break the main loop and terminate the program.
static std::atomic<bool> g_terminate(false);

//////////////////////////////////////////////////
std::string make_channel_name(const std::string& name) {
  const std::string defaultPrefix{"DRIVING_COMMAND"};
  if (name.empty()) {
    return defaultPrefix;
  }
  return defaultPrefix + "_" + name;
}

//////////////////////////////////////////////////
/// \brief Function callback executed when a SIGINT or SIGTERM signals are
/// captured. This is used to exit the program smoothly.
/// \param[in] _signal The signal captured.
void signal_handler(int _signal) {
  if (_signal == SIGINT || _signal == SIGTERM)
    g_terminate = true;
}

/// \brief A wrapper to execute the Drake simulator as a back-end.
/// This class exposes some of the functionality of the simulator via Ignition
/// Transport services (e.g.: play/pause/step the simulation). At the same time,
/// it has to periodically advance the simulation a certain amount of time.
/// Finally, it notifies any given state changes to the outside world via
/// Ignition Transport messages. All these operations occur in a main loop with
/// the following structure:
///
/// * Process incoming messages: All incoming requestes are queued
///   asynchronously when received and in this step we process all of them.
///   This request might involve flag the simulation as paused, insert a new
///   model, remove an existing model, etc.
///
/// * Step the simulation: We advance the simulation a certain amount of time.
///   The amount of time could be the default time step (if we are in play
///   mode), a custom time step (if we are externally stepping) or no time at
///   all if the simulation is paused.
///
/// * Notify changes in the simulation. The simulation will send messages over a
///   a well-known topic every time that a relevant simulation state changed
///   occur. E.g.: When the simulation is paused or a model is removed. These
///   notifications should be used by all the Visualizer instances to update its
///   state/widgets in order to keep them in sync with the back-end.
class AutomotiveDemo {

  /// \brief Default constructor.
  /// \param[in] _timeStep The slot of time (seconds) simulated in each
  /// simulation step.
  public: AutomotiveDemo(const double _timeStep)
      : timeStep(_timeStep) {
    // Enable to resolve relative path to resources on AddPriusSimpleCar
    drake::AddResourceSearchPath(std::string(
      std::getenv("DRAKE_INSTALL_PATH")) + "/share/drake");

    this->simulator = std::make_unique<AutomotiveSimulator<double>>();

    SimpleCarState<double> state;
    state.set_y(0.0);
    this->simulator->AddPriusSimpleCar("0", make_channel_name("0"), state);

    // Advertise the service for controlling the simulation.
    this->node.Advertise(this->kControlService,
      &AutomotiveDemo::OnWorldControl, this);

    // Advertise the topic for publishing notifications.
    this->notificationsPub = this->node.Advertise<ignition::msgs::WorldControl>(
      this->kNotificationsTopic);

    simulator->Start();
  }

  //////////////////////////////////////////////////
  /// \brief Default destructor.
  public: virtual ~AutomotiveDemo() = default;

  //////////////////////////////////////////////////
  /// \brief Execute the main simulation loop.
  public: void Run() {
    while (!g_terminate) {

      // 0. Start a timer to measure the time we spend doing tasks.
      auto stepStart = std::chrono::steady_clock::now();

      // 1. Process incoming messages (requests).
      {
        std::lock_guard<std::mutex> lock(this->mutex);
        this->ProcessIncomingMessages();
      }

      // 2. Step the simulator (if needed).
      if (!this->IsPaused()) {
        simulator->StepBy(this->timeStep);
      } else if (this->StepRequested()) {
        simulator->StepBy(this->CustomTimeStep());
      }

      // Remove any custom step request.
      this->SetStepRequested(false);

      // 3. Process outgoing messages (notifications).
      {
        std::lock_guard<std::mutex> lock(this->mutex);
        this->SendOutgoingMessages();
      }

      // 4. Stop the timer.
      auto stepEnd = std::chrono::steady_clock::now();

      // 5. Wait for the remaining time of this step.
      auto stepElapsed = stepEnd - stepStart;
        std::this_thread::sleep_for(
          std::chrono::microseconds(static_cast<long int>(this->timeStep * 1e6))
          - stepElapsed);
    }
  }

  //////////////////////////////////////////////////
  /// \brief Process all pending incoming messages.
  private: void ProcessIncomingMessages() {
    while (!this->incomingMsgs.empty()) {
      auto nextMsg = this->incomingMsgs.front();
      this->incomingMsgs.pop();

      // Process the message.
      this->ProcessWorldControlMessage(nextMsg);
    }
  }

  //////////////////////////////////////////////////
  /// \brief Send all outgoing messages.
  private: void SendOutgoingMessages() {
    while (!this->outgoingMsgs.empty()) {
      auto nextMsg = this->outgoingMsgs.front();
      this->outgoingMsgs.pop();

      // Send the message.
      this->notificationsPub.Publish(nextMsg);
    }
  }

  //////////////////////////////////////////////////
  /// \brief Process one WorldControl message.
  /// \param[in] _msg The message
  private: void ProcessWorldControlMessage(
      const ignition::msgs::WorldControl &_msg) {
    if (_msg.has_pause()) {
      this->SetPaused(_msg.pause());
    } else if (_msg.has_step() && _msg.step()) {
      this->SetStepRequested(true);
      this->SetCustomTimeStep(this->TimeStep());
    } else if (_msg.has_multi_step() && _msg.multi_step() > 0u) {
      this->SetStepRequested(true);
      this->SetCustomTimeStep(this->TimeStep() * _msg.multi_step());
    } else {
      ignwarn << "Ignoring world control message" << std::endl;
    }
  }

  //////////////////////////////////////////////////
  /// \brief Service used to receive world control messages.
  /// \param[in] _req The request.
  /// \param[out] _rep The response (unused).
  /// \param[out] _result The result of the service.
  private: void OnWorldControl(const ignition::msgs::WorldControl &_req,
      ignition::msgs::Boolean &_rep, bool &_result) {

    {
      // Just queue the message.
      std::lock_guard<std::mutex> lock(this->mutex);
      this->incomingMsgs.push(_req);
    }

    _result = true;
  }

  /// \brief Get the default time step.
  /// \return The default time step.
  private: double TimeStep() const {
    return this->timeStep;
  }

  //////////////////////////////////////////////////
  /// \brief Set the default time step.
  /// \param[in] _timeStep The new time step.
  private: void SetTimeStep(const double _timeStep) {
    this->timeStep = _timeStep;
  }

  //////////////////////////////////////////////////
  /// \brief Get whether the simulation is paused or not.
  /// \return True when the simulation is paused or false otherwise.
  private: bool IsPaused() const {
    return this->paused;
  }

  //////////////////////////////////////////////////
  /// \brief Pause/unpause the simulation.
  /// \param[in] _paused True for paused, false for unpaused.
  private: void SetPaused(const bool _paused) {
    this->paused = _paused;
  }

  //////////////////////////////////////////////////
  /// \brief Whether an external step was requested or not.
  /// \return True if requested or false otherwise.
  private: bool StepRequested() const {
    return this->stepRequested;
  }

  //////////////////////////////////////////////////
  /// \brief Set whether an external step is requested or not.
  /// \param[in] _stepRequested True when a step is requested.
  private: void SetStepRequested(const bool _stepRequested) {
    this->stepRequested = _stepRequested;
  }

  //////////////////////////////////////////////////
  /// \brief Get the custom time step for an external step.
  private: double CustomTimeStep() const {
    return this->customTimeStep;
  }

  //////////////////////////////////////////////////
  /// \brief Set the custom time step for an external step.
  /// \param[in] _timeStep The custom time step.
  private: void SetCustomTimeStep(const double _timeStep) {
    this->customTimeStep = _timeStep;
  }

  /// \brief The service offered to control the simulation.
  private: const std::string kControlService = "/world_control";

  /// \brief The topic used to publish notifications.
  private: const std::string kNotificationsTopic = "/notifications";

  /// \brief The time (seconds) that we simulate in each simulation step.
  private: double timeStep = 0.001;

  /// \brief The time (seconds) that we simulate in a custom step requested
  /// externally.
  private: double customTimeStep = 0.001;

  /// \brief A pointer to the Drake simulator.
  private: std::unique_ptr<AutomotiveSimulator<double>> simulator = nullptr;

  /// \brief Whether the simulation is paused or not.
  private: bool paused = false;

  /// \brief Whether an external step was requested or not.
  private: bool stepRequested = false;

  /// \brief A mutex to avoid races.
  private: std::mutex mutex;

  /// \brief A queue for storing the incoming messages (requests).
  private: std::queue<ignition::msgs::WorldControl> incomingMsgs;

  /// \brief A queue for storing the outgoing messages (notifications).
  private: std::queue<ignition::msgs::WorldControl> outgoingMsgs;

  /// \brief An Ignition Transport node used for communication.
  private: ignition::transport::Node node;

  /// \brief An Ignition Transport publisher for sending notifications.
  private: ignition::transport::Node::Publisher notificationsPub;
};

}  // namespace backend
}  // namespace delphyne

//////////////////////////////////////////////////
int main(int argc, char* argv[]) {
  // Install a signal handler for SIGINT and SIGTERM.
  std::signal(SIGINT,  delphyne::backend::signal_handler);
  std::signal(SIGTERM, delphyne::backend::signal_handler);

  delphyne::backend::AutomotiveDemo demo(0.001);
  demo.Run();
}
