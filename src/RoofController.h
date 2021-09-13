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
  BeltTension,
  SafetyTaskNotRunning
};

enum MotorState {
  Stopped,
  RunningUp,
  RunningDown,
};

class RoofController {
public:
  RoofController();
  virtual ~RoofController(){};

  void initialize();
  void updateStateMachine();
  void setCodeReceived(bool code);
  void setLastCommand(Trigger);
  void setError(Error);
  void resetError(Error);
  void clearErrors();
  bool hasError(Error);
  void setMotorState(MotorState);

  void setUpdatePeriod(uint32_t period) { updatePeriod = period; };
  uint32_t getUpdatePeriod() { return updatePeriod; };

private:
  friend struct States;

  hsm::StateMachine stateMachine;
  Sense::Task stateMachineTask;

  bool codeReceived   = false;
  Trigger lastTrigger = None;
  std::bitset<sizeof(Error)> errors;
  uint32_t updatePeriod = 1000;
};

struct States {

  struct BaseState : StateWithOwner<RoofController> {
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
      if (Owner().hasError(SafetyTaskNotRunning)) {
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
      if (IsInInnerState<Moving_To_WaitingForCode>()) {
        return SiblingTransition<WaitingForCode>();
      }

      if (IsInInnerState<Moving_To_Idling>()) {
        return SiblingTransition<Idling>();
      }

      if (Owner().hasError(VehicleRunnning)) {
        return SiblingTransition<Idling>();
      }

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

      return NoTransition();
    };
  };

  // Intermediate state to transition from Opening or Closing to WaitingForCode
  // Moving state immediately transitions to WaitingForCode when this state is active
  // This construction is needed to execute onExit of Moving state
  struct Moving_To_WaitingForCode : BaseState {
    DEFINE_HSM_STATE(Moving_To_WaitingForCode)
  };

  // Intermediate state to transition from Opening or Closing to Idling
  struct Moving_To_Idling : BaseState {
    DEFINE_HSM_STATE(Moving_To_Idling)
  };

  struct OpeningRoof : BaseState {
    DEFINE_HSM_STATE(OpeningRoof)

    void OnEnter(const uint32_t timeoutMs = 0) {
      timeout   = timeoutMs;
      startTime = millis();
      Owner().setMotorState(RunningUp);
    }
    void OnExit() override {
      Owner().setMotorState(Stopped);
    }

    Transition GetTransition() override {
      if (Owner().hasError(MaxLimit)) {
        return SiblingTransition<Moving_To_WaitingForCode>();
      }

      if (timeout) {
        if ((millis() - startTime > timeout)) {
          timeout = 0;
          return SiblingTransition<Moving_To_Idling>();
        }
      }

      return NoTransition();
    };

  private:
    unsigned long timeout   = 0;
    unsigned long startTime = 0;
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
        return SiblingTransition<Moving_To_WaitingForCode>();
      }
      if (Owner().hasError(BeltTension)) {
        return SiblingTransition<OpeningRoof>(2000);
      }
      return NoTransition();
    };
  };
};
