package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import frc.robot.Constants.RobotType;
import frc.robot.subsystems.swerve.DriveConstants;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;
import org.littletonrobotics.junction.AutoLogOutput;

public class RobotSimState {

  private SwerveDriveSimulation driveSimulation = null;

  public SwerveDriveSimulation getDriveSimulation() {
    return driveSimulation;
  }

  // Intake sim variables
  @AutoLogOutput(key = "RobotSimState/Intake Has Coral")
  private boolean intakeHasCoral;

  @AutoLogOutput(key = "RobotSimState/Distance From Coral Ejection")
  private Distance distanceFromEject;

  public void coralIntaked() {
    intakeHasCoral = true;
    distanceFromEject = Meters.of(-.5); // FIXME: change to const val
  }

  public void updateCoralPosition(LinearVelocity updateVelocity) {
    if (!intakeHasCoral) return; // if we don't have coral, just exit

    distanceFromEject =
        distanceFromEject.minus(
            updateVelocity.times(Time.ofBaseUnits(0.02, Second))); // update the distance

    if (distanceFromEject.compareTo(Meters.of(0)) <= 0) { // if no more distance to go
      intakeHasCoral = false;
      // spawn in the coral
      if (Constants.getRobotType() == RobotType.SIM) {

        Pose3d currentCoralEjectionPose = new Pose3d(); // FIXME: make this a real pose3d
        SimulatedArena.getInstance()
            .addGamePieceProjectile(
                new ReefscapeCoralOnFly(
                    driveSimulation.getSimulatedDriveTrainPose().getTranslation(),
                    currentCoralEjectionPose.toPose2d().getTranslation(),
                    driveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                    driveSimulation.getSimulatedDriveTrainPose().getRotation(),
                    Units.Meters.of(currentCoralEjectionPose.getZ()),
                    updateVelocity,
                    currentCoralEjectionPose.getRotation().getMeasureY()));
      }
    }
  }

  public Distance getDistanceFromEject() {
    return distanceFromEject;
  }

  public boolean getIntakeHasCoral() {
    return intakeHasCoral;
  }

  private static RobotSimState instance;

  public static RobotSimState getInstance() {
    if (Constants.getRobotType() != RobotType.SIM) {
      System.err.println("ROBOT SHOULD NOT ACCESS SIM STATE IN NORMAL MODE");
    }
    if (instance == null) {
      instance = new RobotSimState();
    }
    return instance;
  }

  public RobotSimState() {
    driveSimulation =
        new SwerveDriveSimulation(
            DriveConstants.mapleSimConfig, RobotState.getInstance().getEstimatedPose());
  }
}
