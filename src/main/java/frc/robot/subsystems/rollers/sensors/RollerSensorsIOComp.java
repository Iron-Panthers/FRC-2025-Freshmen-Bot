package frc.robot.subsystems.rollers.sensors;

import edu.wpi.first.wpilibj.DigitalInput;
// TODO: make a sensor class - different subdirectory- RobotState should have sensors

public class RollerSensorsIOComp implements RollerSensorsIO {
  // FIXME; pretty sure rio DIO pullup
  private final DigitalInput intakeSensor = new DigitalInput(7);

  @Override
  public void updateInputs(RollerSensorsIOInputs inputs) {
    inputs.intakeDetected = !intakeSensor.get();
  }
}
