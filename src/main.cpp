#include <Arduino.h>

#include "RoofController.h"

RoofController controller{};

void setup() {
  Serial.begin(115200);
  log_i("Starting HSM Experiment");
  controller.initialize();
}

uint32_t cnt;
void loop() {
  delay(1000);
  cnt++;

  if (cnt == 10) {
    controller.clearErrors();
    log_i("====================== Scenario: Enable, Roof Open, Cancel, Roof Open, Max limit");
    log_d("\t\t\t===> Code received");
    controller.setCodeReceived(true);
  }

  if (cnt == 15) {
    log_d("\t\t\t===> Roof Open pressed");
    controller.setLastCommand(OpenRoof);
  }

  if (cnt == 20) {
    log_d("\t\t\t===> Cancel pressed");
    controller.setLastCommand(Cancel);
  }
  if (cnt == 22) {
    log_d("\t\t\t===> Roof Open pressed");
    controller.setLastCommand(OpenRoof);
  }
  if (cnt == 25) {
    log_d("\t\t\t===> Max Limit reached");
    controller.setError(MaxLimit);
  }

  if (cnt == 40) {
    controller.clearErrors();
    log_i("====================== Scenario: Enable, Roof Close, Min limit");
    log_d("\t\t\t===> Code received");
    controller.setCodeReceived(true);
  }

  if (cnt == 45) {
    log_d("\t\t\t===> Roof Closed pressed");
    controller.setLastCommand(CloseRoof);
  }

  if (cnt == 55) {
    log_d("\t\t\t===> Min Limit reached");
    controller.setError(MinLimit);
  }

  if (cnt == 60) {
    controller.clearErrors();
    controller.setError(MinLimit);
    log_i("====================== Scenario: Enable, Roof Close with Min limit already reached");
    log_d("\t\t\t===> Code received");
    controller.setCodeReceived(true);
  }

  if (cnt == 65) {
    log_d("\t\t\t===> Roof Closed pressed");
    controller.setLastCommand(CloseRoof);
  }

  if (cnt == 70) {
    controller.clearErrors();
    log_i("====================== Scenario: Enable, Roof Open, Vehicle started");
    log_d("\t\t\t===> Code received");
    controller.setCodeReceived(true);
  }

  if (cnt == 75) {
    log_d("\t\t\t===> Roof Open pressed");
    controller.setLastCommand(OpenRoof);
  }

  if (cnt == 80) {
    log_d("\t\t\t===> Vehicle started");
    controller.setError(VehicleRunnning);
  }

  if (cnt == 85) {
    controller.clearErrors();
    log_i("====================== Scenario: Enable, Roof Close, Belt Tension Error");
    log_d("\t\t\t===> Code received");
    controller.setCodeReceived(true);
  }

  if (cnt == 90) {
    log_d("\t\t\t===> Roof Close pressed");
    controller.setLastCommand(CloseRoof);
  }

  if (cnt == 95) {
    log_d("\t\t\t===> Belt Tension Error");
    controller.setError(BeltTension);
  }

  if (cnt == 100) {
    log_i("====================== All Scenarios executed =============================");
  }
}