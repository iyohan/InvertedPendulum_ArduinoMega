# InvertedPendulum_ArduinoMega

Arduino Mega 2560 based inverted pendulum project with milestone sketches showing the development path from motor and IMU tests to the final balancing controller.

## Project Structure

- `final/`
  - Final working sketch.
- `stages/`
  - Milestone sketches kept as representative development steps.

## Final Sketch

- `final/final_balancing_controller/final_balancing_controller.ino`
  - Final balancing controller version.

## Milestones

- `stages/milestone01_motor_oneside/milestone01_motor_oneside.ino`
  - Single motor and encoder speed test.
- `stages/milestone02_imu_parser/milestone02_imu_parser.ino`
  - EBIMU serial parsing test.
- `stages/milestone03_dual_motor_encoder/milestone03_dual_motor_encoder.ino`
  - Dual motor and encoder measurement on Arduino Mega.
- `stages/milestone04_sensor_motor_integration/milestone04_sensor_motor_integration.ino`
  - First integration of IMU input with motor control.
- `stages/milestone05_first_balance_control/milestone05_first_balance_control.ino`
  - First version with state-based balance control logic.
- `stages/milestone06_pre_final_tuning/milestone06_pre_final_tuning.ino`
  - Pre-final tuning version before the final controller.

## Notes

- The sketches are organized so that each folder name matches its `.ino` file name for Arduino IDE compatibility.
- The final and milestone sketches target Arduino Mega 2560.
- IMU-related sketches use `Serial3` for EBIMU communication.
