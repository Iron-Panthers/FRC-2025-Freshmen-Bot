package frc.robot.subsystems.rollers;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.generic_subsystems.rollers.*;
import frc.robot.subsystems.rollers.intake.Intake;
import frc.robot.subsystems.rollers.sensors.*;
import org.littletonrobotics.junction.Logger;

public class Rollers extends SubsystemBase {

  public enum RollerState {
    IDLE,
    INTAKE,
    FORCE_INTAKE,
    EJECT_TOP,
    EJECT_L1,
    EJECT_L2,
    EJECT_L3,
    // TODO: make EJECT_L4
    HOLD
  }

  private final Intake intake;
  private final RollerSensorsIO sensorsIO;

  // TODO:Have an alternative to force intake/cleaner implementation
  // private double ejectTime = 0;
  private double intakeTime = 0;
  private double timeSinceStopped = 0;

  private RollerState targetState = RollerState.IDLE;
  private RollerSensorsIOInputsAutoLogged sensorsInputs = new RollerSensorsIOInputsAutoLogged();

  public Rollers(Intake intake, RollerSensorsIO sensorsIO) {
    this.intake = intake;
    this.sensorsIO = sensorsIO;
  }

  @Override
  public void periodic() {
    sensorsIO.updateInputs(sensorsInputs);
    Logger.processInputs("RollerSensors", sensorsInputs);

    switch (targetState) {
      case IDLE -> {
        intake.setVoltageTarget(Intake.Target.IDLE);
      }
      case INTAKE -> {
        intake.setVoltageTarget(Intake.Target.INTAKE);
        if (intakeSensorsTriggered()) {
          this.targetState = RollerState.HOLD;
        }
      }
      case FORCE_INTAKE -> {
        intakeTime += 0.02;
        intake.setVoltageTarget(Intake.Target.INTAKE);
        if (intakeTime > 0.5) {
          this.targetState = RollerState.INTAKE;
          intakeTime = 0;
        }
      }
      case HOLD -> {
        intake.setVoltageTarget(Intake.Target.HOLD);
      }
      case EJECT_TOP -> {
        intake.setVoltageTarget(Intake.Target.EJECT_TOP);
      }
      case EJECT_L1 -> {
        intake.setVoltageTarget(Intake.Target.EJECT_L1);
      }
      case EJECT_L2 -> {
        intake.setVoltageTarget(Intake.Target.EJECT_L2);
      }
      case EJECT_L3 -> {
        intake.setVoltageTarget(Intake.Target.EJECT_L3);
      }
        // you should never reach this
      default -> {
        intake.setVoltageTarget(Intake.Target.IDLE);
        System.err.println("PLEASE ADD INTAKE STATE, YOU ABSOLUTE BUFFOON");
      }
    }
    if (intakeSensorsTriggered()) {
      timeSinceStopped += 0.02;
    } else {
      timeSinceStopped = 0;
    }

    intake.periodic();

    Logger.recordOutput("Rollers/TargetState", targetState);
  }

  public RollerState getTargetState() {
    return targetState;
  }

  public void setTargetState(RollerState targetState) {
    this.targetState = targetState;
  }

  public Command setTargetCommand(RollerState target) {
    return new InstantCommand(
        () -> {
          this.targetState = target;
        });
  }

  public boolean intakeSensorsTriggered() {
    return sensorsInputs.intakeDetected;
  }

  public boolean readyToRaise() {
    return intakeSensorsTriggered() && timeSinceStopped > 0.1;
  }
}
