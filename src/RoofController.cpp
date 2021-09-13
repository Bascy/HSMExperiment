#include "RoofController.h"

void runStateMachine(void* pvParameters) {
  RoofController* controller = static_cast<RoofController*>(pvParameters);
  controller->updateStateMachine();
  delay(controller->getUpdatePeriod());
}

RoofController::RoofController() : stateMachineTask({runStateMachine, "RoofController", 2048, this, 5, 1, true, false, true, true}){};

void RoofController::initialize() {
  stateMachine.Initialize<RoofStates::WaitingForCode>(this);
  stateMachine.SetDebugInfo("StateController", TraceLevel::Basic);
  stateMachineTask.resumeTask();
}

void RoofController::updateStateMachine() {
  stateMachine.ProcessStateTransitions();
  stateMachine.UpdateStates();
}

void RoofController::setCodeReceived(bool code) {
  codeReceived = code;
};

void RoofController::setLastCommand(Trigger trigger) {
  lastTrigger = trigger;
};

void RoofController::setError(Error error) {
  errors[(int)error] = true;
};

void RoofController::resetError(Error error) {
  errors[(int)error] = false;
};

void RoofController::clearErrors() {
  errors.reset();
};

bool RoofController::hasError(Error error) {
  return errors[(int)error];
};

void RoofController::setMotorState(MotorState state) {
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
};
