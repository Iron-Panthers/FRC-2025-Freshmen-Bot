package frc.robot;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.RobotType;
import frc.robot.subsystems.swerve.DriveConstants;
import edu.wpi.first.math.MathUtil;


public class RobotSimState {

    private SwerveDriveSimulation driveSimulation = null;

    public SwerveDriveSimulation getDriveSimulation() {
        return driveSimulation;
    }

    // Intake sim variables
    private boolean intakeHasCoral;
    private double distanceFromEject;


    public void coralIntaked(){
        intakeHasCoral = true;
        distanceFromEject = 2;
    }

    public void updateCoralPosition(double updateVelocity) {
        if (!intakeHasCoral)
            return; // if we don't have coral, just exit

        distanceFromEject -= updateVelocity * 0.02; // update the distance

        if (distanceFromEject <= 0) { // if no more distance to go
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
                                        Units.MetersPerSecond.of(-1),
                                        currentCoralEjectionPose.getRotation().getMeasureY()));

            }
        }
    }

    public double getDistanceFromEject() {
        return distanceFromEject;
    }

    public boolean getIntakeHasCoral() {
        return intakeHasCoral;
    }

    private static RobotSimState instance;

    public static RobotSimState getInstance() {
        if (Constants.getRobotType() != RobotType.SIM) { // FIXME: I don't like it
            return null;
        }
        if (instance == null) {
            instance = new RobotSimState();
        }
        return instance;
    }

    public RobotSimState() {
        driveSimulation = new SwerveDriveSimulation(
                DriveConstants.mapleSimConfig, RobotState.getInstance().getEstimatedPose());
    }
}
