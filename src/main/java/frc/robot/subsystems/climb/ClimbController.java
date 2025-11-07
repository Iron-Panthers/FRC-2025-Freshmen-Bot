package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.generic_subsystems.superstructure.GenericSuperstructure.ControlMode;
import frc.robot.subsystems.climb.climb_pivot.Climb;
import frc.robot.subsystems.climb.climb_pivot.Climb.ClimbTarget;
import frc.robot.subsystems.climb.climb_sensors.ClimbSensorsIO;
import frc.robot.subsystems.climb.climb_sensors.ClimbSensorsIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

public class ClimbController extends SubsystemBase {

  public enum ClimbState {
    BOTTOM,
    TOP,
    CLEAR,
    STOW
  }

  private final Climb climb;
  private final ClimbSensorsIO climbSensorsIO;

  private ClimbState targetState = ClimbState.STOW;
  private ClimbSensorsIOInputsAutoLogged climbSensorsInputs = new ClimbSensorsIOInputsAutoLogged();
  /** Creates a new ClimbController. */
  public ClimbController(Climb climb, ClimbSensorsIO climbSensorsIO) {
    this.climb = climb;
    this.climbSensorsIO = climbSensorsIO;

    // climb.setOffset();
    // climb.setPositionTarget(ClimbTarget.STOW);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    climbSensorsIO.updateInputs(climbSensorsInputs);
    Logger.processInputs("ClimbSensors", climbSensorsInputs);

    switch (targetState) {
      case BOTTOM -> {
        climb.setPositionTarget(ClimbTarget.BOTTOM);
      }
      case TOP -> {
        climb.setPositionTarget(ClimbTarget.TOP);
      }
      case CLEAR -> {
        climb.setPositionTarget(ClimbTarget.CLEAR);
      }
      case STOW -> {
        climb.setPositionTarget(ClimbTarget.STOW);
      }
      default -> {
        climb.setPositionTarget(Climb.ClimbTarget.STOW);
        System.err.println("PLEASE ADD CLIMB STATE, YOU ABSOLUTE BUM");
      }
    }

    climb.periodic();
    Logger.recordOutput("Climb/TargetState", targetState);
  }

  public Command setPositionTargetCommand(ClimbState targetState) {
    return new InstantCommand(
        () -> {
          setClimbTarget(targetState);
        });
  }

  // Flick the climb to let coral fall out
  public Command clearCoral() {
    return new SequentialCommandGroup(
        // Wait until we get to the clear position
        new FunctionalCommand(
            () -> {
              climb.setPositionTarget(ClimbTarget.CLEAR);
            },
            () -> {},
            (e) -> {},
            climb::reachedTarget),

        // Then just go back up to stow
        new InstantCommand(
            () -> {
              climb.setPositionTarget(ClimbTarget.STOW);
            }));
  }

  public boolean climbHitCage() {
    return climb.hitCage();
  }

  public ClimbTarget getClimbTarget() {
    return climb.getPositionTarget();
  }

  public void setClimbTarget(ClimbState targetState) {
    this.targetState = targetState;
  }

  public void setStopped(boolean stopped) {
    climb.setControlMode(ControlMode.STOP);
  }

  public boolean climbSensorsTriggered() {
    return climbSensorsInputs.climbDetected;
  }
}
