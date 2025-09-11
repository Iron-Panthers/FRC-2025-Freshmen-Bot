package frc.robot.subsystems.swerve.controllers;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

import static frc.robot.subsystems.swerve.DriveConstants.PID_AUTOALIGN_CONSTANTS;
import static frc.robot.subsystems.swerve.DriveConstants.TRAJECTORY_CONFIG;

import java.util.function.Supplier;

import org.dyn4j.geometry.Rotation;
import org.littletonrobotics.junction.AutoLogOutput;

//1. go to notion and put these notes on there
        /*2. headingcontroller basically takes an angle and goes to the target angle based on calculations
        TeleopController basically takes the yaw of the joystick and translates that into an output in the drive class
        now you have to figure out what to do with the PIDAutoAlignController class:
        what you want to do is you have the position and you just need to get to another position using pid
         * you can calculate stuff in order to get the correct pid */


public class PIDAutoAlignController {
    
    //supplies the position values
    private ProfiledPIDController controller;
    private Supplier<Rotation2d> positionSupplier;

    //target position
    private Pose2d targetPosition;

    
    public PIDAutoAlignController() {
        this.positionSupplier = positionSupplier;
        this.targetPosition = targetPosition;
        
        //setting up the ProfiledPIDCawontroller
        controller = new ProfiledPIDController(
            PID_AUTOALIGN_CONSTANTS.kP(),
            0,
            PID_AUTOALIGN_CONSTANTS.kD(),
            new Constraints(PID_AUTOALIGN_CONSTANTS.maxVelocity(),
            PID_AUTOALIGN_CONSTANTS.maxAcceleration()),
            Constants.PERIODIC_LOOP_SEC);

        
        //calculate how to get to the desired position
        
        
        //update the values 
        
        //log your data in advantage kit

    }
}
