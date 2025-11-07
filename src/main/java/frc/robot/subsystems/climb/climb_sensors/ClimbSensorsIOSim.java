package frc.robot.subsystems.climb.climb_sensors;

public class ClimbSensorsIOSim implements ClimbSensorsIO {

  @Override
  public void updateInputs(ClimbSensorsIOInputs inputs) {
    inputs.climbDetected = false;
  }
}
