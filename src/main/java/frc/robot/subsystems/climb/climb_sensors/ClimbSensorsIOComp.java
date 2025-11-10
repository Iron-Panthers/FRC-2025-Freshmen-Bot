package frc.robot.subsystems.climb.climb_sensors;

import edu.wpi.first.wpilibj.DigitalInput;
// TODO: make a sensor class - different subdirectory- RobotState should have sensors

public class ClimbSensorsIOComp implements ClimbSensorsIO {
  // FIXME; pretty sure rio DIO pullup
  private final DigitalInput intakeSensor = new DigitalInput(0);

  @Override
  public void updateInputs(ClimbSensorsIOInputs inputs) {
    inputs.climbDetected = !intakeSensor.get();
  }
}
