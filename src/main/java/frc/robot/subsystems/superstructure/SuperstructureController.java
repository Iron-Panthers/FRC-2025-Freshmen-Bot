package frc.robot.subsystems.superstructure;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.generic_subsystems.superstructure.GenericSuperstructure.ControlMode;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.elevator.Elevator.ElevatorTarget;
import frc.robot.subsystems.superstructure.pivot.Pivot;
import frc.robot.subsystems.superstructure.pivot.Pivot.PivotTarget;

public class SuperstructureController extends SubsystemBase {
  public enum SuperstructureState {
    SETUP_L4, // Setting up in L4
    SCORE_L4, // Scoring in L4
    SETUP_L3, // Setting up in L3
    SCORE_L3, // Scoring in L3
    L2, // Scoring in L2
    TOP, // Apex
    STOW, // Going to the lowest position
    ZERO; // Zero the motor
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
  }

  @Override
  public void periodic() {
    if (!stop) {
      switch (currentState) { // switch on the target state
         case L2 -> {
          elevator.setPositionTarget(ElevatorTarget.L2);
          pivot.setPositionTarget(PivotTarget.L2);

          // check for state transitions
          if (this.superstructureReachedTarget()) {
            if (targetState != currentState) {
                if (targetState != currentState) {
                setCurrentState(SuperstructureState.TOP);
              }
            }
          }
        }

        case SETUP_L3 -> {
          if (elevator.getPosition() > 32) {
            pivot.setPositionTarget(PivotTarget.SETUP_L3);
          }
          elevator.setPositionTarget(ElevatorTarget.L3);

          // check for state transitions
          if (this.superstructureReachedTarget() && targetState != currentState) {
            switch (targetState) {
              case SCORE_L3 -> setCurrentState(SuperstructureState.SCORE_L3);
              default -> setCurrentState(SuperstructureState.TOP);
            }
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