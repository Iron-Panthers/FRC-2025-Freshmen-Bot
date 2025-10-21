// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.Mode;
import frc.robot.commands.VibrateHIDCommand;
import frc.robot.subsystems.canWatchdog.CANWatchdog;
import frc.robot.subsystems.canWatchdog.CANWatchdogIO;
import frc.robot.subsystems.canWatchdog.CANWatchdogIOComp;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.ClimbController;
import frc.robot.subsystems.climb.ClimbIOSim;
import frc.robot.subsystems.climb.ClimbIOTalonFX;
import frc.robot.subsystems.rgb.RGB;
import frc.robot.subsystems.rgb.RGBIO;
import frc.robot.subsystems.rgb.RGBIOCANdle;
import frc.robot.subsystems.rollers.Rollers;
import frc.robot.subsystems.rollers.Rollers.RollerState;
import frc.robot.subsystems.rollers.intake.Intake;
import frc.robot.subsystems.rollers.intake.IntakeIO;
import frc.robot.subsystems.rollers.intake.IntakeIOSim;
import frc.robot.subsystems.rollers.intake.IntakeIOTalonFX;
import frc.robot.subsystems.rollers.sensors.RollerSensorsIOComp;
import frc.robot.subsystems.superstructure.SuperstructureController;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.elevator.ElevatorIOSim;
import frc.robot.subsystems.superstructure.pivot.Pivot;
import frc.robot.subsystems.superstructure.pivot.PivotIOSim;
import frc.robot.subsystems.swerve.Drive;
import frc.robot.subsystems.swerve.DriveConstants;
import frc.robot.subsystems.swerve.GyroIO;
import frc.robot.subsystems.swerve.GyroIOPigeon2;
import frc.robot.subsystems.swerve.GyroIOSim;
import frc.robot.subsystems.swerve.ModuleIO;
import frc.robot.subsystems.swerve.ModuleIOTalonFXReal;
import frc.robot.subsystems.swerve.ModuleIOTalonFXSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import java.util.function.BooleanSupplier;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // private SendableChooser<Command> autoChooser;
  private LoggedDashboardChooser<Command> autoChooser;

  private final CommandXboxController driverA = new CommandXboxController(0);
  private final CommandXboxController driverB = new CommandXboxController(1);

  private Drive swerve;
  private Vision vision;
  private RGB rgb;
  private CANWatchdog canWatchdog;
  private SuperstructureController superstructureController;
  private ClimbController climbController;
  private SwerveDriveSimulation driveSimulation = null;
  private Intake intake;
  private Elevator elevator;
  private RollerSensorsIOComp rollerSensors;
  private Rollers rollers;

  public RobotContainer() {

    if (Constants.getRobotMode() != Mode.REPLAY) {
      switch (Constants.getRobotType()) {
        case COMP -> {
          swerve =
              new Drive(
                  new GyroIOPigeon2(),
                  new ModuleIOTalonFXReal(DriveConstants.MODULE_CONFIGS[0]),
                  new ModuleIOTalonFXReal(DriveConstants.MODULE_CONFIGS[1]),
                  new ModuleIOTalonFXReal(DriveConstants.MODULE_CONFIGS[2]),
                  new ModuleIOTalonFXReal(DriveConstants.MODULE_CONFIGS[3]));
          //   vision = new Vision(new VisionIOPhotonvision(4), new VisionIOPhotonvision(5));
          rgb = new RGB(new RGBIOCANdle());
          canWatchdog = new CANWatchdog(new CANWatchdogIOComp(), rgb);
          intake = new Intake(new IntakeIOTalonFX());
          rollerSensors = new RollerSensorsIOComp();
          climbController = new ClimbController(new Climb(new ClimbIOTalonFX()));
        }
        case SIM -> {
          SimulatedArena.getInstance()
              .addDriveTrainSimulation(RobotSimState.getInstance().getDriveSimulation());
          swerve =
              new Drive(
                  new GyroIOSim(
                      RobotSimState.getInstance().getDriveSimulation().getGyroSimulation()),
                  new ModuleIOTalonFXSim(
                      DriveConstants.MODULE_CONFIGS[0],
                      RobotSimState.getInstance().getDriveSimulation().getModules()[0]),
                  new ModuleIOTalonFXSim(
                      DriveConstants.MODULE_CONFIGS[1],
                      RobotSimState.getInstance().getDriveSimulation().getModules()[1]),
                  new ModuleIOTalonFXSim(
                      DriveConstants.MODULE_CONFIGS[2],
                      RobotSimState.getInstance().getDriveSimulation().getModules()[2]),
                  new ModuleIOTalonFXSim(
                      DriveConstants.MODULE_CONFIGS[3],
                      RobotSimState.getInstance().getDriveSimulation().getModules()[3]));
          // vision =
          //     new Vision(
          //         new VisionIOPhotonvisionSim(4, driveSimulation::getSimulatedDriveTrainPose),
          //         new VisionIOPhotonvisionSim(5, driveSimulation::getSimulatedDriveTrainPose));
          superstructureController =
              new SuperstructureController(
                  new Elevator(new ElevatorIOSim()), new Pivot(new PivotIOSim()));

          climbController = new ClimbController(new Climb(new ClimbIOSim()));
          SimulatedArena.getInstance().resetFieldForAuto();
          intake = new Intake(new IntakeIOSim());
        }
      }
    }

    if (swerve == null) {
      swerve =
          new Drive(
              new GyroIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {});
    }
    if (vision == null) {
      vision = new Vision(new VisionIO() {}, new VisionIO() {});
    }

    if (canWatchdog == null) {
      canWatchdog = new CANWatchdog(new CANWatchdogIO() {}, rgb);
    }

    if (rgb == null) {
      rgb = new RGB(new RGBIO() {});
    }
    if (intake == null) {
      intake = new Intake(new IntakeIO() {});
    }
    if (rollerSensors == null) {
      rollerSensors = new RollerSensorsIOComp() {};
    }

    rollers = new Rollers(intake, rollerSensors);

    nameCommands();
    configureAutos();
    configureBindings();
  }

  public void containerMatchStarting() {
    // runs when match starts
    canWatchdog.matchStarting();
  }

  /** Use this method to define the named commands for all of the autos */
  private void nameCommands() {
    // Register Command Names in this method
  }

  private void configureBindings() {
    // -----Driver Controls-----
    swerve.setDefaultCommand(
        swerve
            .run(
                () -> {
                  swerve.driveTeleopController(
                      -driverA.getLeftY(),
                      -driverA.getLeftX(),
                      driverA.getLeftTriggerAxis() - driverA.getRightTriggerAxis(),
                      // In SIM-2025, the commented line of code below is present, but here there
                      // isn't a superstructure variable yet.
                      // superstructure.getElevatorPosition() > 3 ? 3 :
                      DriveConstants.DRIVE_CONFIG.maxLinearAcceleration());

                  if (Math.abs(driverA.getLeftTriggerAxis()) > 0.1
                      || Math.abs(driverA.getRightTriggerAxis()) > 0.1) {
                    swerve.clearHeadingControl();

                    // In SIM-2025, the "true" is a variable called autoAngle that is never changed
                    // from true. I'm not sure what to do with this exactly...
                  } else if (true) {

                    determineSwerveTarget();
                  }
                })
            .withName("Drive Teleop"));

    driverA.start().onTrue(swerve.zeroGyroCommand());

    driverA.a().onTrue(new InstantCommand(() -> swerve.smartZeroGyro()));

    // Make rollers move
    driverA.x().onTrue(rollers.setTargetCommand(RollerState.INTAKE));
    driverA.y().onTrue(rollers.setTargetCommand(RollerState.HOLD));

    driverA.b().onTrue(new InstantCommand(() -> RobotSimState.getInstance().coralIntaked()));
    driverA.a().onTrue(rollers.setTargetCommand(RollerState.EJECT_L1));
  }

  private void configureAutos() {
    RobotConfig robotConfig;
    try {
      robotConfig = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      e.printStackTrace();
      robotConfig = null;
    }

    var passRobotConfig = robotConfig; // workaround

    BooleanSupplier flipAlliance =
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red
          // alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        };

    AutoBuilder.configure(
        () -> RobotState.getInstance().getEstimatedPose(),
        (pose) -> RobotState.getInstance().resetPose(pose),
        () -> swerve.getRobotSpeeds(),
        (speeds) -> {
          swerve.setTrajectorySpeeds(speeds);
        },
        DriveConstants.HOLONOMIC_DRIVE_CONTROLLER,
        passRobotConfig,
        flipAlliance,
        swerve);

    autoChooser =
        new LoggedDashboardChooser<Command>("Auto Chooser", AutoBuilder.buildAutoChooser());
    SmartDashboard.putData("Auto Chooser", autoChooser.getSendableChooser());
  }

  public Command getAutoCommand() {
    return AutoBuilder.buildAuto("R L4 (3) (EDC)"); // HACK: Replace once we get auto logging
  }

  // runs when auto starts
  public void autoInit() {
    // Smart zero the robot
    CommandScheduler.getInstance().schedule(new InstantCommand(() -> swerve.smartZeroGyro()));
  }

  // runs when teleop starts
  public void teleopInit() {
    CommandScheduler.getInstance()
        .schedule(new ParallelCommandGroup(new VibrateHIDCommand(driverB.getHID(), 5, .5)));

    // vibrate controller at 30 seconds left
    CommandScheduler.getInstance()
        .schedule(
            new WaitCommand(105)
                .andThen(
                    new ParallelCommandGroup(new VibrateHIDCommand(driverB.getHID(), 3, 0.4))));
  }

  public void updateDashboardStatus() {
    // TODO: Define all of the dashboard outputs here
    SmartDashboard.putString("Current Auto", autoChooser.get().getName());
  }

  // #region Helper functions May wrote or copied from elsewhere cuz she's a little plagarizer >:(

  /**
   * Wraps around the value of a double to 0 - 360.
   *
   * @param angle
   * @return A double with a value from 0 - 360 (but not including 360).
   */
  public static double doubleToDegrees(double angle) {
    return (angle % 360 + 360) % 360;
  }

  /**
   * Outputs the relative angular difference of the two angles given. (I rewrote this because the
   * old one seemed broken to me. I'm not really sure though??? Can someone check how it's meant to
   * work please?)
   *
   * @param currentAngle
   * @param newAngle
   * @return A double representing an angle in degrees from -180 to 180.
   */
  public static double relativeAngularDifference(double currentAngle, double newAngle) {
    return (doubleToDegrees(newAngle - currentAngle) + 180) % 360 - 180;
  }

  /**
   * Snaps an angle towards the closest value in REEF_SNAP_ANGLES.
   *
   * @param targetHeading A rotation2d representing the target angle
   * @return A Rotation2d representing the angle closest to the target angle
   */
  public static Rotation2d calculateSnapTargetHeading(Rotation2d targetHeading) {

    targetHeading = targetHeading.rotateBy(Rotation2d.kPi); // because back of robot

    double closest = DriveConstants.REEF_SNAP_ANGLES[0];
    for (double snap : DriveConstants.REEF_SNAP_ANGLES) {

      if (Math.abs(relativeAngularDifference(targetHeading.getDegrees(), snap))
          < Math.abs(relativeAngularDifference(targetHeading.getDegrees(), closest))) {

        closest = snap;
      }
    }

    return new Rotation2d(Math.toRadians(closest));
  }

  /** Determines what direction to snap the swerve target to. */
  public void determineSwerveTarget() {

    // Repeated variables:
    Translation2d robotPosition = RobotState.getInstance().getEstimatedPose().getTranslation();
    Rotation2d robotAngleToReef = robotPosition.minus(DriveConstants.REEF_TRANSLATION2D).getAngle();
    boolean isTeamRed =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;

    // Snap towards right corner when close
    if (robotPosition.getDistance(DriveConstants.RIGHT_CORNER) < 3) {
      swerve.setTargetHeading(new Rotation2d(Math.toRadians(232)));

      // Snap towards left corner when close
    } else if (robotPosition.getDistance(DriveConstants.LEFT_CORNER) < 3) {
      swerve.setTargetHeading(new Rotation2d(Math.toRadians(128)));

      // Snap towards the reef when close
    } else if (robotPosition.getDistance(DriveConstants.REEF_TRANSLATION2D) < 2) {

      swerve.setTargetHeading(
          isTeamRed
              ? calculateSnapTargetHeading(robotAngleToReef)
              : FlippingUtil.flipFieldRotation(calculateSnapTargetHeading(robotAngleToReef)));

      // Snap towards climb zone when close and in climb phase
    } else if (MathUtil.isNear(DriveConstants.CLIMB_ZONE_CENTER.getX(), robotPosition.getX(), 2)
        && MathUtil.isNear(DriveConstants.CLIMB_ZONE_CENTER.getY(), robotPosition.getY(), 2)
    /* && superstructure.getTargetState() == SuperstructureState.CLIMB */
    /* TO-DO: Once the state system is implemented, uncomment this line! */ ) {
      swerve.setTargetHeading(new Rotation2d(Math.PI / 2));

      // If none of the previous conditions match, snap towards reef
    } else {
      swerve.setTargetHeading(
          robotAngleToReef.minus(isTeamRed ? Rotation2d.kPi : Rotation2d.kZero));
    }
  }
  // #endregion

  public void updateSimulation() {
    if (Constants.getRobotMode() != Constants.Mode.SIM) return;

    SimulatedArena.getInstance().simulationPeriodic();
    Logger.recordOutput(
        "FieldSimulation/RobotPosition",
        RobotSimState.getInstance().getDriveSimulation().getSimulatedDriveTrainPose());
    Logger.recordOutput(
        "FieldSimulation/Coral", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
    Logger.recordOutput(
        "FieldSimulation/Algae", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
  }
}
