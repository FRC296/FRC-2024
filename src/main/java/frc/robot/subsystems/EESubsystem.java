package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class EESubsystem extends SubsystemBase {

    TalonSRX m_topBelt = new TalonSRX(3);
    TalonSRX m_bottomBelt = new TalonSRX(2);
    TalonSRX m_topShooter = new TalonSRX(4);
    TalonSRX m_bottomShooter = new TalonSRX(1);

    DigitalInput beamBreak = new DigitalInput(0);

    XboxController controller;

    private Boolean hasNote = true;

    public Boolean getHasNote() {
        return hasNote;
    }

    // Constructor
    public EESubsystem(XboxController controller) {
        m_topShooter.setInverted(true);
        m_bottomBelt.follow(m_topBelt);

        this.controller = controller;
    }

    // Intake the note
    public void intake() {
        if(!hasNote) {
            m_topBelt.set(ControlMode.PercentOutput, 1);
            if(beamBreak.get()) {
                hasNote = true;
                controller.setRumble(RumbleType.kLeftRumble, 1);
            }
        } else {
            m_topBelt.set(ControlMode.PercentOutput, 0);
            // intakeCommand.end(false);
        }
    }

    public void overrideIntake() {
        m_topBelt.set(ControlMode.PercentOutput, 1);
    }

    // Intake as a command
    public RunCommand intakeCommand = new RunCommand(() -> {
        intake();
    },
    this);

    // Spin up the shooter
    public void revUp() {
        m_topShooter.set(ControlMode.PercentOutput, 1);
        m_bottomShooter.set(ControlMode.PercentOutput, 1);
    }

    // Spin up the shooter as a command
    public InstantCommand revUpCommand = new InstantCommand(() -> {
        this.revUp();
    },
    this);

    // Shoot at the speaker (without spin up time)
    public InstantCommand quickShotCommand = new InstantCommand(() -> {
        m_topShooter.set(ControlMode.PercentOutput, 1);
        m_bottomShooter.set(ControlMode.PercentOutput, 1);
        Timer.delay(0.25);
        m_topBelt.set(ControlMode.PercentOutput, 1);
        Timer.delay(0.65);
        m_topBelt.set(ControlMode.PercentOutput, 0);
        m_bottomShooter.set(ControlMode.PercentOutput, 0);
        m_topShooter.set(ControlMode.PercentOutput, 0);
        hasNote = false;
    },
    this);
    
    // Shoot at the speaker (with spin up time)
    public InstantCommand simpleShotCommand = new InstantCommand(() -> {    
        m_topShooter.set(ControlMode.PercentOutput, 1);
        m_bottomShooter.set(ControlMode.PercentOutput, 1);
        Timer.delay(0.85);
        m_topBelt.set(ControlMode.PercentOutput, 1);
        Timer.delay(0.65);
        m_topBelt.set(ControlMode.PercentOutput, 0);
        m_bottomShooter.set(ControlMode.PercentOutput, 0);
        m_topShooter.set(ControlMode.PercentOutput, 0);
        hasNote = false;
    },
    this);

    // Shoot at the amp
    public InstantCommand ampShotCommand = new InstantCommand(() -> {
        m_topShooter.set(ControlMode.PercentOutput, 0.2);
        m_bottomShooter.set(ControlMode.PercentOutput, 0.7);
        Timer.delay(0.25);
        m_topBelt.set(ControlMode.PercentOutput, 1);
        Timer.delay(0.65);
        m_topBelt.set(ControlMode.PercentOutput, 0);
        m_bottomShooter.set(ControlMode.PercentOutput, 0);
        m_topShooter.set(ControlMode.PercentOutput, 0);
        hasNote = false;
    },
    this);

    // Stop the shooter and belt
    public RunCommand stopCommand = new RunCommand(() -> {
        m_topBelt.set(ControlMode.PercentOutput, 0);
        m_bottomShooter.set(ControlMode.PercentOutput, 0);
        m_topShooter.set(ControlMode.PercentOutput, 0);
    },
    this);

    public InstantCommand instantStop = new InstantCommand(() -> {
        m_topBelt.set(ControlMode.PercentOutput, 0);
        m_bottomShooter.set(ControlMode.PercentOutput, 0);
        m_topShooter.set(ControlMode.PercentOutput, 0);
    },
    this);

    // Reverse the belt to remove jams
    public InstantCommand reverseCommand = new InstantCommand(() -> {
        m_topBelt.set(ControlMode.PercentOutput, -1);
        m_topShooter.set(ControlMode.PercentOutput, -1);
        m_bottomShooter.set(ControlMode.PercentOutput, -1);
        Timer.delay(0.5);
        m_topBelt.set(ControlMode.PercentOutput, 0);
        m_bottomShooter.set(ControlMode.PercentOutput, 0);
        m_topShooter.set(ControlMode.PercentOutput, 0);
    },
    this);

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Has Note", hasNote);
    }
}
