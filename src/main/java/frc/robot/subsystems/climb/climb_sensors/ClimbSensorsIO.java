package frc.robot.subsystems.climb.climb_sensors;

import org.littletonrobotics.junction.AutoLog;

public interface ClimbSensorsIO {
  @AutoLog
  class ClimbSensorsIOInputs {
    public boolean climbDetected = false;
  }

  public default void updateInputs(ClimbSensorsIOInputs inputs) {}
}
