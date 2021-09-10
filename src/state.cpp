#include "state.h"

/**
 * Task to read check every 0.5 sec if nodes can be added or must be removed
 */
void runStateMachine(void* pvParameters) {
  StateController* controller = static_cast<StateController*>(pvParameters);
  controller->updateStateMachine();
  delay(controller->getUpdatePeriod());
}

StateController::StateController() : stateMachineTask({runStateMachine, "stateController", 2048, this, 1, 1, true, false, true}){

                                     };

void StateController::initialize() {
  stateMachine.Initialize<States::WaitingForCode>(this);
  stateMachine.SetDebugInfo("StateController", TraceLevel::Basic);
  stateMachineTask.resumeTask();
}

void StateController::setUpdatePeriod(uint32_t period) {
  log_d("\t\tSetting update period to %u", period);
  updatePeriod = period;
}

uint32_t StateController::getUpdatePeriod() {
  return updatePeriod;
}

void StateController::updateStateMachine() {
  stateMachine.ProcessStateTransitions();
  stateMachine.UpdateStates();
}

void StateController::setCodeReceived(bool code) {
  codeReceived = code;
};

void StateController::setLastCommand(Trigger trigger) {
  lastTrigger = trigger;
};

void StateController::setError(Error error) {
  errors[(int)error] = true;
};

void StateController::resetError(Error error) {
  errors[(int)error] = false;
};

void StateController::clearErrors() {
  errors.reset();
};

bool StateController::hasError(Error error) {
  return errors[(int)error];
};

void StateController::setMotorState(MotorState state) {
  switch (state) {
    case Stopped:
      log_d("\t\tMotor stopped");
      break;
    case RunningUp:
      log_d("\t\tMotor running up");
      break;
    case RunningDown:
      log_d("\t\tMotor running down");
      break;
  }
  motorState = state;
};
