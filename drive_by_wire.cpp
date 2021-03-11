
#include <my_rtos.hpp>
#include "sensor_interface.hpp"
#include "actuator_interface.hpp"

// Declares stack space for threads (implemented in rtos somewhere)
static constexpr size_t kInputsThreadStackSize = 16;  // Stack space in bytes
static THREAD_STACK_SPACE(stk_command_input_thread, kInputsThreadStackSize);

static constexpr size_t kOutputsThreadStackSize = 32;  // Stack space in bytes
static THREAD_STACK_SPACE(stk_actuator_outputs_thread, kOutputsThreadStackSize);

enum class DriveMode : int32_t {
  FORWARD = 0,
  REVERSE = 1,
  NEUTRAL = 2
};

typedef struct commandinputs {
  int16_t steering_wheel_position;
  int16_t brake_pedal_position;
  int16_t throttle_pedal_position;
  DriveMode drive_mode_selector;
} CommandInputs;

typedef struct actuatoroutputs{
  int16_t steering_actuator_cmd;
  uint16_t brake_actuator_cmd;
  uint16_t throttle_actuator_cmd;
  DriveMode drive_mode_actuator_cmd;
} ActuatorOutputs;

// Global variable to hold most recent command from remote controller
static CommandInputs g_most_recent_command_inputs;

/* Some forward Declarations of functions we're using (they're implemented elsewhere) */

/**
 *  Blocking function that waits for command inputs over a lossy link, and then returns a pointer to the command inputs.
 * @param timeout_milliseconds is the maximum time to wait for commands before returning.
 * 
 * @return a pointer to a commands struct. NULL if the function timed out before receiving a valid command.
 */
CommandInputs* ReadInputsFromDriveByWireControlInterface(const int timeout_milliseconds);

/**
 * Sends CAN commands to actuators
 * @param outputs_cmd the outputs to command to the actuator
 * 
 * @return true if the CAN commands were all sent successfully.
 */
bool SendOutputsToActuators(ActuatorOutputs& const outputs_cmd);



// The main program
int main(void)
{
  rtosInitSystem(); // Start up the RTOS

  // Start the threads, passing in the stack space for each thread to use as well as the priority of the thread.
  // NOTE: This rtos uses a preemptive scheduler, so the highest priority thread is given control as soon as it's ready.
  rtosThreadStart(ReadCommandsThread, stk_command_input_thread, NORMAL_PRIORITY);
  rtosThreadStart(SendOutputsThread, stk_actuator_outputs_thread, NORMAL_PRIORITY + 1);  // Higher priority

  // Blink the heartbeat LED forever.
  while (true) {
    rtosToggleGpioPad(GPIOC, GPIOC_ONBOARD_HEARTBEAT_LED);  // Onboard heartbeat

    rtosTheadSleepMilliseconds(500);
  }
}

// Thread #1
void ReadCommandsThread(void)
{
  // Read in commands from a remote controller
  while (true) {
    CommandInputs* command_inputs = ReadInputsFromDriveByWireControlInterface(10);
    g_most_recent_command_inputs = *command_inputs;
    
    rtosTheadSleepMilliseconds(10);
  }
}

// Thread #2
void SendOutputsThread(void)
{
  // Send commands to the actuators every 10ms
  while (true) {
    ActuatorOutputs outputs;
    outputs.steering_actuator_cmd = g_most_recent_command_inputs.steering_wheel_position;
    outputs.brake_actuator_cmd = g_most_recent_command_inputs.brake_pedal_position;
    outputs.throttle_actuator_cmd = g_most_recent_command_inputs.throttle_pedal_position;
    outputs.drive_mode_selector_cmd = g_most_recent_command_inputs.drive_mode_selector;

    SendOutputsToActuators(outputs);

    rtosTheadSleepMilliseconds(10);
  }
}