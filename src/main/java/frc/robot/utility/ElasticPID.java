package frc.robot.utility;

import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ElasticPID {
  SetSlot0Lambda lambda;

  GravityTypeValue gravityTypeValue;
  
  String folderName;

  // Constructor should take in lambda for setSlot0 function
  public ElasticPID(SetSlot0Lambda setSlot0Function, GravityTypeValue gravityTypeValue, String folderName) {
    SmartDashboard.putNumber(folderName + "/kP", 0);
    SmartDashboard.putNumber(folderName + "/kI", 0);
    SmartDashboard.putNumber(folderName + "/kD", 0);
    SmartDashboard.putNumber(folderName + "/kS", 0);
    SmartDashboard.putNumber(folderName + "/kV", 0);
    SmartDashboard.putNumber(folderName + "/kA", 0);
    SmartDashboard.putNumber(folderName + "/kG", 0);
    SmartDashboard.putNumber(folderName + "/motionMagicAcceleration", 0);
    SmartDashboard.putNumber(folderName + "/motionMagicCruiseVelocity", 0);
    SmartDashboard.putNumber(folderName + "/motionMagicJerk", 0);

    this.folderName = folderName;
    this.gravityTypeValue = gravityTypeValue;
    this.lambda = setSlot0Function;
  }
  // testtest

  // Periodic function
  public void periodic() {
    this.lambda.setSlot0(
        SmartDashboard.getNumber(folderName + "/kP", 0),
        SmartDashboard.getNumber(folderName + "/kI", 0),
        SmartDashboard.getNumber(folderName + "/kD", 0),
        SmartDashboard.getNumber(folderName + "/kS", 0),
        SmartDashboard.getNumber(folderName + "/kV", 0),
        SmartDashboard.getNumber(folderName + "/kA", 0),
        SmartDashboard.getNumber(folderName + "/kG", 0),
        SmartDashboard.getNumber(folderName + "/motionMagicAcceleration", 0),
        SmartDashboard.getNumber(folderName + "/motionMagicCruiseVelocity", 0),
        SmartDashboard.getNumber(folderName + "/motionMagicJerk", 0),
        gravityTypeValue);
  }

  // Interface for lambda
  public interface SetSlot0Lambda {
    public void setSlot0(
      double kP,
      double kI,
      double kD,
      double kS,
      double kV,
      double kA,
      double kG,
      double motionMagicAcceleration,
      double motionMagicCruiseVelocity,
      double motionMagicJerk,
      GravityTypeValue gravityTypeValue);
      }
}
