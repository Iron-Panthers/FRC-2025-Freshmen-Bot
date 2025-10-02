package frc.robot.subsystems.swerve.controllers;

import static frc.robot.subsystems.swerve.DriveConstants.PID_AUTOALIGN_CONSTANTS;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.robot.Constants;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class PIDAutoAlignController {

  // supplies the position values
  private ProfiledPIDController xController;
  private ProfiledPIDController yController;
  private Supplier<Pose2d> positionSupplier;

  private Pose2d translCurrPosition;
  // target position
  private Pose2d targetPosition;
  private Pose2d translTargetPosition;
  private Pose2d startPosition;
  private double xVel;
  private double yVel;
  private final Supplier<Rotation2d> yawSupplier;

  public PIDAutoAlignController(
      Supplier<Pose2d> positionSupplier, Supplier<Rotation2d> yawSupplier, Pose2d targetPosition) {
    this.startPosition = positionSupplier.get();
    this.positionSupplier = positionSupplier;
    this.targetPosition = targetPosition;
    this.translTargetPosition =
        new Pose2d(
            targetPosition.getX() - startPosition.getX(),
            targetPosition.getY() - startPosition.getY(),
            new Rotation2d());
    this.yawSupplier = yawSupplier;

    // setting up the ProfiledPIDCawontroller
    xController =
        new ProfiledPIDController(
            PID_AUTOALIGN_CONSTANTS.kP(),
            PID_AUTOALIGN_CONSTANTS.kI(),
            PID_AUTOALIGN_CONSTANTS.kD(),
            new Constraints(
                PID_AUTOALIGN_CONSTANTS.maxVelocity(), PID_AUTOALIGN_CONSTANTS.maxAcceleration()),
            Constants.PERIODIC_LOOP_SEC);

    yController =
        new ProfiledPIDController(
            PID_AUTOALIGN_CONSTANTS.kP(),
            PID_AUTOALIGN_CONSTANTS.kI(),
            PID_AUTOALIGN_CONSTANTS.kD(),
            new Constraints(
                PID_AUTOALIGN_CONSTANTS.maxVelocity(), PID_AUTOALIGN_CONSTANTS.maxAcceleration()),
            Constants.PERIODIC_LOOP_SEC);
  }
  // calculate how to get to the desired position
  public void calculateLinearMovement() {
    // basically this code says that
    // yVel    (y-y1)
    // ---- =  ------
    // xVel    (x-x1)
    double dy = targetPosition.getY() - positionSupplier.get().getY();
    double dx = targetPosition.getX() - positionSupplier.get().getX();
    double desiredSlope = (dy / dx);
    if (dx > dy) { // will take longer to get to X than to Y
      xVel = xController.calculate(translCurrPosition.getX(), translTargetPosition.getX());
      yController.calculate(translCurrPosition.getY(), translTargetPosition.getY());
      yVel = xVel * desiredSlope;
    } else { // will take longer to get to Y than to X
      yVel = yController.calculate(translCurrPosition.getY(), translTargetPosition.getY());
      xController.calculate(translCurrPosition.getX(), translTargetPosition.getX());
      xVel = yVel / desiredSlope;
    }
  }

  // update the values
  public ChassisSpeeds update() {
    translCurrPosition =
        new Pose2d(
            positionSupplier.get().getX() - startPosition.getX(),
            positionSupplier.get().getY() - startPosition.getY(),
            new Rotation2d());
    Logger.recordOutput("Swerve/PID/SetpointPosition", xController.getSetpoint().position);
    Logger.recordOutput("Swerve/PID/SetpointVelocity", xController.getSetpoint().velocity);
    calculateLinearMovement();
    return ChassisSpeeds.fromFieldRelativeSpeeds(-xVel, -yVel, 0, yawSupplier.get());
  }
  // log your data in advantage kit
  public Pose2d getTargetPosition() {
    return targetPosition;
  }

  public double getXVel() {
    return -xVel;
  }

  public double getYVel() {
    return -yVel;
  }

  public void setTargetPosition(Pose2d targetPosition) {
    startPosition = positionSupplier.get();
    this.targetPosition = targetPosition;
    translTargetPosition =
        new Pose2d(
            targetPosition.getX() - startPosition.getX(),
            targetPosition.getY() - startPosition.getY(),
            new Rotation2d());
  }
}
