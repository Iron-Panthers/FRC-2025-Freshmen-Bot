package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.generic_subsystems.superstructure.GenericSuperstructure.ControlMode;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.elevator.Elevator.ElevatorTarget;
import frc.robot.subsystems.superstructure.elevator.ElevatorConstants;
import frc.robot.subsystems.superstructure.pivot.Pivot;
import frc.robot.subsystems.superstructure.pivot.Pivot.PivotTarget;
import org.littletonrobotics.junction.Logger;

public class SuperstructureController extends SubsystemBase {
  public enum SuperstructureState {
    L4, // Scoring in L4
    L3, // Scoring in L3
    L2, // Scoring in L2
    L1,
    TOP, // Apex
    ZERO, // Zero the motor
    INTAKE;
  }

  private boolean stop = false;
  private SuperstructureState currentState;
  private SuperstructureState targetState;

  private final Elevator elevator;
  private final Pivot pivot;

  public SuperstructureController(Elevator elevator, Pivot pivot) {
    this.elevator = elevator;
    this.pivot = pivot;
    pivot.setPositionTarget(PivotTarget.STOW);
    elevator.setPositionTarget(ElevatorTarget.BOTTOM);

    pivot.setParent(elevator);
    currentState = SuperstructureState.INTAKE;
  }

  @Override
  public void periodic() {
    if (!stop) {
      switch (currentState) { // switch on the target state
        case L1 -> {
          pivot.setPositionTarget(PivotTarget.L1);
          elevator.setPositionTarget(ElevatorTarget.L1);
        }
        case L2 -> {
          pivot.setPositionTarget(PivotTarget.L2);
          elevator.setPositionTarget(ElevatorTarget.L2);
        }
        case L3 -> {
          pivot.setPositionTarget(PivotTarget.L3);
          elevator.setPositionTarget(ElevatorTarget.L3);
        }
        case L4 -> {
          pivot.setPositionTarget(PivotTarget.L4);
          elevator.setPositionTarget(ElevatorTarget.L4);
        }
        case TOP -> {
          pivot.setPositionTarget(PivotTarget.TOP);
          elevator.setPositionTarget(ElevatorTarget.TOP);
        }
        case INTAKE -> {
          pivot.setPositionTarget(PivotTarget.INTAKE);
          elevator.setPositionTarget(ElevatorTarget.INTAKE);
        }
        case ZERO -> {
          pivot.setPositionTarget(PivotTarget.ZERO);
          elevator.setZeroing(true);
          if (elevator.getFilteredSupplyCurrentAmps()
              > ElevatorConstants.ZEROING_VOLTAGE_THRESHOLD) {
            elevator.setOffset();
            elevator.setControlMode(ControlMode.POSITION);
            elevator.setZeroing(false);
          }
        }
      }
    } else {
      elevator.setControlMode(ControlMode.STOP);
      pivot.setControlMode(ControlMode.STOP);
    }

    elevator.periodic();
    pivot.periodic();

    Logger.recordOutput("Superstructure/TargetState", targetState);
    Logger.recordOutput("Superstructure/CurrentState", currentState);
    Logger.recordOutput("Superstructure/Elevator reached target", elevator.reachedTarget());
    Logger.recordOutput("Superstructure/Pivot reached target", pivot.reachedTarget());
    Logger.recordOutput("Superstructure/Reached Target", superstructureReachedTarget());
    Logger.recordOutput("Superstructure/Mechanism Positions/Elevator", elevator.getDisplayPose3d());
    Logger.recordOutput("Superstructure/Mechanism Positions/Pivot", pivot.getDisplayPose3d());
  }

  // Target state getter and setter
  public void setTargetState(SuperstructureState superstructureState) {
    this.stop = false;
    this.targetState = superstructureState;
  }

  public SuperstructureState getTargetState() {
    return targetState;
  }

  // Current state getter and setter
  public void setCurrentState(SuperstructureState superstructureState) {
    this.stop = false;
    this.currentState = superstructureState;
  }

  public SuperstructureState getCurrentState() {
    return currentState;
  }

  public void setStopped(boolean stopped) {
    this.stop = stopped;
  }

  public boolean getStopped() {
    return stop;
  }

  // go to target state command factory
  public Command goToStateCommand(SuperstructureState superstructureState) {
    return new FunctionalCommand(
        () -> {
          setTargetState(superstructureState);
        },
        () -> {},
        (e) -> {},
        () -> {
          return currentState == targetState && superstructureReachedTarget();
        },
        this);
  }

  /**
   * Get the position of the elevator
   *
   * @return the position of the elevator
   */
  public double getElevatorPosition() {
    return elevator.getPosition();
  }

  /**
   * Get the position of the pivot
   *
   * @return the position of the pivot
   */
  public double getPivotPosition() {
    return pivot.getPosition();
  }

  /**
   * Get the supply current of the elevator
   *
   * @return the supply current of the elevator
   */
  public double getElevatorSupplyCurrentAmps() {
    return elevator.getSupplyCurrentAmps();
  }
  /**
   * Get the supply current of the pivot
   *
   * @return the supply current of the pivot
   */
  public double getPivotSupplyCurrentAmps() {
    return pivot.getSupplyCurrentAmps();
  }

  /**
   * @return a boolean that says whether or not both of our mechanisms have finished zeroing
   */
  public boolean notZeroing() {
    return !elevator.isZeroing();
  }

  /**
   * @return if both subsystems in the superstructure have reached their target
   */
  public boolean superstructureReachedTarget() {
    boolean output =
        (elevator.reachedTarget()
            && pivot.reachedTarget()
            && currentState != SuperstructureState.ZERO);

    return output;
  }
}
