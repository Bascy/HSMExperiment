#pragma once

#include "Arduino.h"
#include "TaskUtils.h"
#include "hsm.h"
#include <bitset>

using namespace hsm;

enum Trigger {
  None,
  OpenRoof,
  CloseRoof,
  Cancel
};

enum Error {
  MaxLimit,
  MinLimit,
  VehicleRunnning,
  BeltTension
};

enum MotorState {
  Stopped,
  RunningUp,
  RunningDown,
};

class StateController {
public:
  StateController();

  void initialize();
  void updateStateMachine();
  void setCodeReceived(bool code);
  void setLastCommand(Trigger);
  void setError(Error);
  void resetError(Error);
  void clearErrors();
  bool hasError(Error);
  void setMotorState(MotorState);

  void setUpdatePeriod(uint32_t);
  uint32_t getUpdatePeriod();

private:
  friend struct States;
  hsm::StateMachine stateMachine;

  bool codeReceived   = false;
  Trigger lastTrigger = None;
  std::bitset<sizeof(Error)> errors;
  MotorState motorState = Stopped;
  Sense::Task stateMachineTask;
  uint32_t updatePeriod = 1000;
};

struct States {

  struct BaseState : StateWithOwner<StateController> {
  };

  struct WaitingForCode : BaseState {
    DEFINE_HSM_STATE(WaitingForCode)

    void OnEnter() override {
      Owner().setCodeReceived(false);
    }

    Transition GetTransition() override {
      if (Owner().codeReceived) {
        return SiblingTransition<Running>();
      }
      return NoTransition();
    };
  };

  struct Running : BaseState {
    DEFINE_HSM_STATE(Running)

    void OnEnter() override {
      log_d("\t\t\tRelease lock");
    }
    void OnExit() override {
      log_d("\t\t\tSet lock");
      Owner().setLastCommand(None);
    }

    Transition GetTransition() override {
      if (Owner().hasError(VehicleRunnning)) {
        return SiblingTransition<WaitingForCode>();
      }

      return InnerEntryTransition<Idling>();
    };
  };

  struct Idling : BaseState {
    DEFINE_HSM_STATE(Idling)

    void OnEnter() override {
      Owner().setLastCommand(None);
    }

    Transition GetTransition() override {
      switch (Owner().lastTrigger) {
        case OpenRoof:
        case CloseRoof:
          return SiblingTransition<Moving>();
        default:
          return NoTransition();
      }
    };
  };

  struct Moving : BaseState {
    DEFINE_HSM_STATE(Moving)

    void OnEnter() {
      Owner().setUpdatePeriod(100);
    }

    void OnExit() {
      Owner().setUpdatePeriod(1000);
    }

    Transition GetTransition() override {
      switch (Owner().lastTrigger) {
        case OpenRoof:
          Owner().lastTrigger = None;
          return InnerTransition<OpeningRoof>(0);
        case CloseRoof:
          Owner().lastTrigger = None;
          return InnerTransition<ClosingRoof>();
        case Cancel:
          Owner().lastTrigger = None;
          return SiblingTransition<Idling>();
        default:
          break;
      }
  
      // This should be done in OpeningRoof, but that somehow skips Moving.OnExit and Idling.OnExit
      // See https://github.com/amaiorano/hsm/issues/14
      if (Owner().hasError(MaxLimit) && Owner().stateMachine.IsInState<OpeningRoof>()) {
        return SiblingTransition<WaitingForCode>();
      }

      // This should be done in ClosingRoof, but that somehow skips Moving.OnExit and Idling.OnExit
      if (Owner().hasError(MinLimit) && Owner().stateMachine.IsInState<ClosingRoof>()) {
        return SiblingTransition<WaitingForCode>();
      }

      if (timeout) {
        if ((millis() - startTime > timeout)) {
          timeout = 0;
          return SiblingTransition<Idling>();
        }
      }

      return NoTransition();
    };

    unsigned long timeout   = 0;
    unsigned long startTime = 0;
  };

  struct OpeningRoof : BaseState {
    DEFINE_HSM_STATE(OpeningRoof)

    void OnEnter(const uint32_t timeoutMs = 0) {
      GetOuterState<Moving>()->timeout   = timeoutMs;
      GetOuterState<Moving>()->startTime = millis();
      Owner().setMotorState(RunningUp);
    }
    void OnExit() override {
      Owner().setMotorState(Stopped);
    }

    Transition GetTransition() override {
      // if (Owner().hasError(MaxLimit)) {
      //   return SiblingTransition<WaitingForCode>();
      // }
      return NoTransition();
    };
  };

  struct ClosingRoof : BaseState {
    DEFINE_HSM_STATE(ClosingRoof)

    void OnEnter() override {
      Owner().setMotorState(RunningDown);
    }
    void OnExit() override {
      Owner().setMotorState(Stopped);
    }

    Transition GetTransition() override {
      if (Owner().hasError(MinLimit)) {
        return SiblingTransition<Idling>();
      }
      if (Owner().hasError(BeltTension)) {
        return SiblingTransition<OpeningRoof>(2000);
      }
      return NoTransition();
    };
  };
};
