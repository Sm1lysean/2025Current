// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// Access the hidden files
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

import java.util.List;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  public static final PneumaticsSS rc_PneumaticsSS = new PneumaticsSS();
  public static final PIDElevSS1 rc_pidElevSS1 = new PIDElevSS1();
  public static final PIDElevSS2 rc_pidElevSS2 = new PIDElevSS2();
  public static final PIDAlgaeSS rc_pidAlgaeSS = new PIDAlgaeSS();
  public static final MotorsSS rc_MotorSS = new MotorsSS();

  // The robot's commands
  public static final PneumaticsC rc_PneumaticsC = new PneumaticsC(rc_PneumaticsSS);
  public static final ElevatorZeroC rc_ElevatorZeroC = new ElevatorZeroC(rc_pidElevSS1);
  public static final AlgaeInC rc_AlgaeInC = new AlgaeInC(rc_MotorSS);
  public static final AlgaeOutC rc_AlgaeOutC = new AlgaeOutC(rc_MotorSS);
  public static final CoralOutC rc_CoralOutC = new CoralOutC(rc_MotorSS);

  // Other instantiations
  public static final PneumaticHub PH = new PneumaticHub(1);

  // The controllers
  public static final CommandXboxController m_driverController = new CommandXboxController(0);
  public static final CommandXboxController m_operatorController = new CommandXboxController(1);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () ->
                m_robotDrive.drive(
                    -MathUtil.applyDeadband(
                        m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(
                        m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(
                        m_driverController.getRightX(), OIConstants.kDriveDeadband),
                    true),
            m_robotDrive));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    // Driver controller button commands
    m_driverController.leftStick().whileTrue(m_robotDrive.setXCommand());
    m_driverController.start().onTrue(m_robotDrive.zeroHeadingCommand());
    // Algae PID
    m_driverController.povUp().onTrue(new AlgaePIDC(rc_pidAlgaeSS, () -> 100));
    m_driverController.povDown().onTrue(new AlgaePIDC(rc_pidAlgaeSS, () -> 0));
    // Coral Control
    m_driverController.a().onTrue(rc_CoralOutC);
    // Operator controller button commands
    m_operatorController.x().onTrue(rc_PneumaticsC);
    m_operatorController.y().onTrue(rc_PneumaticsC);
    // Elevator PID
    m_operatorController.povUp().onTrue(new ElevatorPIDC1(rc_pidElevSS1, () -> 100));
    m_operatorController.povRight().onTrue(new ElevatorPIDC1(rc_pidElevSS1, () -> 50));
    m_operatorController.povDown().onTrue(new ElevatorPIDC1(rc_pidElevSS1, () -> 0));
    m_operatorController.povUp().onTrue(new ElevatorPIDC2(rc_pidElevSS2, () -> 100));
    m_operatorController.povRight().onTrue(new ElevatorPIDC2(rc_pidElevSS2, () -> 50));
    m_operatorController.povDown().onTrue(new ElevatorPIDC2(rc_pidElevSS2, () -> 0));
    m_operatorController.start().onTrue(rc_ElevatorZeroC);
    // Algae Control
    m_operatorController.a().whileTrue(rc_AlgaeInC);
    m_operatorController.b().whileTrue(rc_AlgaeOutC);
  }

  @SuppressWarnings("null")
  public double getSimulationTotalCurrentDraw() {
    // for each subsystem with simulation
    return (Double) null;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(0)),
            config);

    var thetaController =
        new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            exampleTrajectory,
            m_robotDrive::getPose, // Functional interface to feed supplier
            DriveConstants.kDriveKinematics,

            // Position controllers
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            thetaController,
            m_robotDrive::setModuleStates,
            m_robotDrive);

    // Reset odometer to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return new PathPlannerAuto("Auto Test");
  }
}
