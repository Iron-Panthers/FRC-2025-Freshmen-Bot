package frc.robot.subsystems.climb.climb_pivot;

import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import frc.robot.Constants;
import frc.robot.subsystems.canWatchdog.CANWatchdogConstants.CAN;

public class ClimbConstants {
  public static final ClimbConfig CLIMB_CONFIG =
      switch (Constants.getRobotType()) {
        case COMP -> new ClimbConfig(CAN.at(13, "Climb Motor"), 1, 14, 0.481);
        case SIM -> new ClimbConfig(37, 1, 45, 0d);
        default -> new ClimbConfig(0, 1, 0, 0d);
      };

  public static final PIDGains GAINS =
      switch (Constants.getRobotType()) {
        case COMP -> new PIDGains(600, 0, 0, 0, 66.5, 5.714, 0);
        case SIM -> new PIDGains(0.5, 0, 0, 0, 0.1236, 0.0, 0.0);
        default -> new PIDGains(0, 0, 0, 0, 0, 0, 0);
      };

  public static final MotionMagicConfig MOTION_MAGIC_CONFIG =
      switch (Constants.getRobotType()) {
        case COMP -> new MotionMagicConfig(2, 1, 0);
        case SIM -> new MotionMagicConfig(2, 1, 0);
        default -> new MotionMagicConfig(0, 0, 0);
      };

  public record ClimbConfig(int motorID, double reduction, int canCoderID, double canCoderOffset) {}

  public record PIDGains(
      double kP, double kI, double kD, double kS, double kV, double kA, double kG) {}

  public static final GravityTypeValue GRAVITY_TYPE = GravityTypeValue.Arm_Cosine;

  public static final InvertedValue MOTOR_DIRECTION = InvertedValue.CounterClockwise_Positive;
  public static final SensorDirectionValue CANCODER_DIRECTION =
      SensorDirectionValue.CounterClockwise_Positive;

  public static final double POSITION_TARGET_EPSILON = 0.03;

  public record MotionMagicConfig(double acceleration, double cruiseVelocity, double jerk) {}

  // SOFT LIMITS
  public static final double UPPER_EXTENSION_LIMIT = 0.45833333333;

  // CURRENT LIMITS
  public static final double UPPER_VOLT_LIMIT = 12;
  public static final double UPPER_VOLT_LIMIT_CLIMBING = 3;
  public static final double LOWER_VOLT_LIMIT = -12;
  public static final double SUPPLY_CURRENT_LIMIT = 30;

  // INDUCTION SENSOR
  public static final int INDUCTION_PORT_NUMBER = 6;

  // PHYSICAL CONSTANTS
  public static record ClimbPhysicalConstants(
      double momentOfInertia,
      double lengthMeters,
      double minAngleRads,
      double maxAngleRads,
      boolean simulateGravity) {}

  public static final ClimbPhysicalConstants PHYSICAL_CONSTANTS =
      switch (Constants.getRobotType()) {
        case SIM -> new ClimbPhysicalConstants(
            0.001, 0.4064, 0, 11 * Math.PI / 12, false); // 11pi/12 = 2.87
        case COMP -> new ClimbPhysicalConstants(
            0.001, 0.4604, 0, 11 * Math.PI / 12, false); // weight in kg
      };
}
