// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.fasterxml.jackson.core.sym.Name;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.EESubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;


/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive;
  public final EESubsystem m_eeSubsystem;
  private final ClimberSubsystem m_climberSubsystem;

    // The driver's controller
    XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

    // The auto picker
    private final SendableChooser<Command> autoChooser;

    // Field visualization
    private final Field2d field;

    // The robot's field-relative mode
    Boolean fieldRelative = true;


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // Subsystem instantiation
    m_robotDrive = new DriveSubsystem(m_driverController);
    m_eeSubsystem = new EESubsystem(m_driverController);
    m_climberSubsystem = new ClimberSubsystem();

    // Register Named Commands
    NamedCommands.registerCommand("intake", m_eeSubsystem.intakeCommand.onlyWhile(() -> !m_eeSubsystem.getHasNote()).andThen(m_eeSubsystem.instantStop));
    NamedCommands.registerCommand("aimedShot", (m_robotDrive.alignToTargetCommand.alongWith(m_eeSubsystem.revUpCommand)).andThen(m_eeSubsystem.quickShotCommand));
    NamedCommands.registerCommand("SetX", m_robotDrive.setXCommand);
    NamedCommands.registerCommand("revUp", m_eeSubsystem.revUpCommand);
    NamedCommands.registerCommand("simpleShot", m_eeSubsystem.simpleShotCommand);
    NamedCommands.registerCommand("ampShot", m_eeSubsystem.ampShotCommand);
    NamedCommands.registerCommand("quickShot", m_eeSubsystem.quickShotCommand);
    NamedCommands.registerCommand("align", m_robotDrive.alignToTargetCommand);
    NamedCommands.registerCommand("stopEE", m_eeSubsystem.stopCommand);

    // Build an auto chooser. This will use Commands.none() as the default option.
    autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto Chooser", autoChooser);

    // Field visualization
    field = new Field2d();
    SmartDashboard.putData("Field", field);

        // Logging callback for current robot pose
        PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
            // Do whatever you want with the pose here
            field.setRobotPose(pose);
        });

        // Logging callback for target robot pose
        PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
            // Do whatever you want with the pose here
            field.getObject("target pose").setPose(pose);
        });

        // Logging callback for the active path, this is sent as a list of poses
        PathPlannerLogging.setLogActivePathCallback((poses) -> {
            // Do whatever you want with the poses here
            field.getObject("path").setPoses(poses);
        });

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
                            -MathUtil.applyDeadband(Math.signum(m_driverController.getLeftY())*Math.pow(m_driverController.getLeftY(),OIConstants.kDriveStickPower)*OIConstants.kDriveStickMultiplier * 0.6, OIConstants.kDriveDeadband),
                            -MathUtil.applyDeadband(Math.signum(m_driverController.getLeftX())*Math.pow(m_driverController.getLeftX(),OIConstants.kDriveStickPower)*OIConstants.kDriveStickMultiplier * 0.6, OIConstants.kDriveDeadband),
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
    
    // Set the default command for the EESubsystem
    m_eeSubsystem.setDefaultCommand(m_eeSubsystem.stopCommand);
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
                fieldRelative = true;
            }));
        BButton.onFalse(m_eeSubsystem.stopCommand);

    // On A hold, align to the target
    JoystickButton AButton = new JoystickButton(m_driverController, Button.kA.value);
        AButton.whileTrue(new RunCommand(
            () -> {
                m_robotDrive.alignToTarget();
                m_eeSubsystem.revUp();
            },
            m_eeSubsystem,
            m_robotDrive));

    // X button intake override
    JoystickButton XButton = new JoystickButton(m_driverController, Button.kX.value);
        XButton.whileTrue(new RunCommand(
            () -> {
                m_eeSubsystem.overrideIntake();
            },
            m_eeSubsystem));
        BButton.onFalse(m_eeSubsystem.stopCommand);
        
    // When run bumper is pressed, run the simple shot command
    JoystickButton rightBumper = new JoystickButton(m_driverController, Button.kRightBumper.value);
        rightBumper.onTrue(m_robotDrive.setXCommand);
        rightBumper.onTrue(m_eeSubsystem.simpleShotCommand);

    // When left bumper is pressed, run the amp shot command
    JoystickButton leftBumper = new JoystickButton(m_driverController, Button.kLeftBumper.value);
        leftBumper.onTrue(m_robotDrive.setXCommand);
        leftBumper.onTrue(m_eeSubsystem.ampShotCommand);

    // When the back button is pressed, zero the gyro
    JoystickButton backButton = new JoystickButton(m_driverController, Button.kBack.value);
        backButton.onTrue(new InstantCommand(
            () -> {
                m_robotDrive.zeroHeading();
            },
            m_robotDrive));

    // D pad up and down control the climber
    POVButton dpadUp = new POVButton(m_driverController, 0);
        dpadUp.onTrue(m_climberSubsystem.extendClimberCommand);
    POVButton dpadDown = new POVButton(m_driverController, 180);
        dpadDown.onTrue(m_climberSubsystem.retractClimberCommand);

    JoystickButton startButton = new JoystickButton(m_driverController, Button.kStart.value);
        startButton.onTrue(m_eeSubsystem.reverseCommand);

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
        Command autoCommand = autoChooser.getSelected();
        autoCommand.addRequirements(m_robotDrive, m_eeSubsystem);
        return autoCommand;
    }
}
