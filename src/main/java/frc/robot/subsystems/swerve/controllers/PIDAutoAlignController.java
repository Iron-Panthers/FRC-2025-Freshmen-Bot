package frc.robot.subsystems.swerve.controllers;

import static frc.robot.subsystems.swerve.DriveConstants.PID_AUTOALIGN_CONSTANTS;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.robot.Constants;
import java.util.function.Supplier;

// 1. go to notion and put these notes on there
/*2. headingcontroller basically takes an angle and goes to the target angle based on calculations
TeleopController basically takes the yaw of the joystick and translates that into an output in the drive class
now you have to figure out what to do with the PIDAutoAlignController class:
what you want to do is you have the position and you just need to get to another position using pid
 * you can calculate stuff in order to get the correct pid */

public class PIDAutoAlignController {

  // supplies the position values
  private ProfiledPIDController controller;
  private Supplier<Pose2d> positionSupplier;

  // target position
  private Pose2d targetPosition;
  private double xVel;
  private double yVel;

  public PIDAutoAlignController(Supplier<Pose2d> positionSupplier, Pose2d targetPosition) {
    this.positionSupplier = positionSupplier;
    this.targetPosition = targetPosition;

    // setting up the ProfiledPIDCawontroller
    controller =
        new ProfiledPIDController(
            PID_AUTOALIGN_CONSTANTS.kP(),
            0,
            PID_AUTOALIGN_CONSTANTS.kD(),
            new Constraints(
                PID_AUTOALIGN_CONSTANTS.maxVelocity(), PID_AUTOALIGN_CONSTANTS.maxAcceleration()),
            Constants.PERIODIC_LOOP_SEC);
  }
  // calculate how to get to the desired position
  public void calculateLinearMovement() {
    //basically this code says that
    //yVel    (y-y1)
    //---- =  ------
    //xVel    (x-x1) 
    double desiredSlope = (positionSupplier.get().getY()-targetPosition.getY())/(positionSupplier.get().getX()-targetPosition.getX());
    if(targetPosition.getX()>targetPosition.getY()){ //will take longer to get to X than to Y
      xVel = controller.calculate(positionSupplier.get().getX(), targetPosition.getX());
      yVel = xVel * desiredSlope;
    }else{ //will take longer to get to Y than to X
      yVel = controller.calculate(positionSupplier.get().getY(),targetPosition.getY());
      xVel = yVel/desiredSlope;
    }
  }

  // update the values
  public ChassisSpeeds update() {
    calculateLinearMovement();
    return new ChassisSpeeds(xVel, yVel, 0);
  }
  // log your data in advantage kit
  public Pose2d getTargetPosition(){
    return targetPosition;
  }
  public void setTargetPosition(Pose2d targetPosition) {
    this.targetPosition = targetPosition;
  }
}
