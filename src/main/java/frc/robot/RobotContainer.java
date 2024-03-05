// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
//import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.EESubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;


/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final EESubsystem m_eeSubsystem = new EESubsystem();

    // The driver's controller
    XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

    Boolean fieldRelative = true;


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> {
                if (fieldRelative) {
                    if (m_driverController.getYButton()) {
                        m_robotDrive.drive(
                            -MathUtil.applyDeadband(Math.signum(m_driverController.getLeftY())*Math.pow(m_driverController.getLeftY(),OIConstants.kDriveStickPower)*OIConstants.kDriveStickMultiplier, OIConstants.kDriveDeadband),
                            -MathUtil.applyDeadband(Math.signum(m_driverController.getLeftX())*Math.pow(m_driverController.getLeftX(),OIConstants.kDriveStickPower)*OIConstants.kDriveStickMultiplier, OIConstants.kDriveDeadband),
                            -MathUtil.applyDeadband((m_driverController.getRightX() + m_driverController.getRightTriggerAxis() - m_driverController.getLeftTriggerAxis()) * OIConstants.kRotStickMultiplier, OIConstants.kDriveDeadband),
                        true, true);
                    } else {
                        m_robotDrive.drive(
                            -MathUtil.applyDeadband(Math.signum(m_driverController.getLeftY())*Math.pow(m_driverController.getLeftY(),OIConstants.kDriveStickPower)*OIConstants.kDriveStickMultiplier * 0.5, OIConstants.kDriveDeadband),
                            -MathUtil.applyDeadband(Math.signum(m_driverController.getLeftX())*Math.pow(m_driverController.getLeftX(),OIConstants.kDriveStickPower)*OIConstants.kDriveStickMultiplier * 0.5, OIConstants.kDriveDeadband),
                            -MathUtil.applyDeadband((m_driverController.getRightX() + m_driverController.getRightTriggerAxis() - m_driverController.getLeftTriggerAxis()) * OIConstants.kRotStickMultiplier, OIConstants.kDriveDeadband),
                        true, true);
                    }
                } else {
                m_robotDrive.drive(
                    -MathUtil.applyDeadband(Math.signum(m_driverController.getLeftY())*Math.pow(m_driverController.getLeftY(),OIConstants.kDriveStickPower)*OIConstants.kDriveStickMultiplier * 0.5, OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(Math.signum(m_driverController.getLeftX())*Math.pow(m_driverController.getLeftX(),OIConstants.kDriveStickPower)*OIConstants.kDriveStickMultiplier * 0.5, OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband((m_driverController.getRightX() + m_driverController.getRightTriggerAxis() - m_driverController.getLeftTriggerAxis()) * OIConstants.kRotStickMultiplier, OIConstants.kDriveDeadband),
                false, true);
                }
            },
            m_robotDrive));
    
    // m_eeSubsystem.setDefaultCommand(
    //     // Do nothing by default
    //     new RunCommand(
    //         () -> {
    //             m_eeSubsystem.stop();
    //         },
    //         m_eeSubsystem));
    
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    
    // On B hold, switch to robot-relaive mode and run the intake command on the EESubsystem
    JoystickButton BButton = new JoystickButton(m_driverController, Button.kB.value);
        BButton.whileTrue(new RunCommand(
            () -> {
                m_eeSubsystem.intake();
                fieldRelative = false;
            },
            m_eeSubsystem));
        BButton.whileFalse(new RunCommand(
            () -> {
                m_eeSubsystem.stop();
                fieldRelative = true;
            },
            m_eeSubsystem));

    // On A hold, align to target and run the shoot command on the EESubsystem
    JoystickButton AButton = new JoystickButton(m_driverController, Button.kA.value);
        AButton.whileTrue(new RunCommand(
            () -> {
                m_eeSubsystem.shoot();
                m_robotDrive.alignToTarget();
            },
            m_eeSubsystem));

    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // // Create config for trajectory
    // TrajectoryConfig config = new TrajectoryConfig(
    //     AutoConstants.kMaxSpeedMetersPerSecond,
    //     AutoConstants.kMaxAccelerationMetersPerSecondSquared)
    //     // Add kinematics to ensure max speed is actually obeyed
    //     .setKinematics(DriveConstants.kDriveKinematics);

    // // An example trajectory to follow. All units in meters.
    // Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
    //     // Start at the origin facing the +X direction
    //     new Pose2d(0, 0, new Rotation2d(0)),
    //     // Pass through these two interior waypoints, making an 's' curve path
    //     List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
    //     // End 3 meters straight ahead of where we started, facing forward
    //     new Pose2d(3, 0, new Rotation2d(0)),
    //     config);

    // var thetaController = new ProfiledPIDController(
    //     AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    // thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
    //     exampleTrajectory,
    //     m_robotDrive::getPose, // Functional interface to feed supplier
    //     DriveConstants.kDriveKinematics,

    //     // Position controllers
    //     new PIDController(AutoConstants.kPXController, 0, 0),
    //     new PIDController(AutoConstants.kPYController, 0, 0),
    //     thetaController,
    //     m_robotDrive::setModuleStates,
    //     m_robotDrive);

    // // Reset odometry to the starting pose of the trajectory.
    // m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // // Run path following command, then stop at the end.
    // return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false, false));
        return null;
    }
}
