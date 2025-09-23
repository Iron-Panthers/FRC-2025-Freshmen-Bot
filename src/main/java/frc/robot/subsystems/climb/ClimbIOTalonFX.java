package frc.robot.subsystems.climb;

import static frc.robot.subsystems.climb.ClimbConstants.*;

import frc.robot.lib.generic_subsystems.superstructure.GenericSuperstructureConfiguration;
import frc.robot.lib.generic_subsystems.superstructure.GenericSuperstructureIOTalonFX;

public class ClimbIOTalonFX extends GenericSuperstructureIOTalonFX implements ClimbIO {

  public ClimbIOTalonFX() {
    super(
        new GenericSuperstructureConfiguration()
            .withID(CLIMB_CONFIG.motorID())
            .withMotorDirection(MOTOR_DIRECTION)
            .withSupplyCurrentLimit(SUPPLY_CURRENT_LIMIT)
            .withReduction(CLIMB_CONFIG.reduction())
            .withUpperVoltageLimit(UPPER_VOLT_LIMIT)
            .withLowerVoltageLimit(LOWER_VOLT_LIMIT)
            .withZeroingVolts(ZEROING_VOLTS)
            .withZeroingOffset(ZEROING_OFFSET)
            .withZeroingVoltageThreshold(ZEROING_VOLTAGE_THRESHOLD)
            .withCANCoderID(CLIMB_CONFIG.canCoderID())
            .withCANCoderDirection(CANCODER_DIRECTION)
            .withCANCoderOffset(CLIMB_CONFIG.canCoderOffset())
            .withUpperExtensionLimit(UPPER_EXTENSION_LIMIT));

    setSlot0(
        GAINS.kP(),
        GAINS.kI(),
        GAINS.kD(),
        GAINS.kS(),
        GAINS.kV(),
        GAINS.kA(),
        GAINS.kG(),
        MOTION_MAGIC_CONFIG.acceleration(),
        MOTION_MAGIC_CONFIG.cruiseVelocity(),
        0,
        GRAVITY_TYPE);
  }
}
