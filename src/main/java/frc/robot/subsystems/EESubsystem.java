package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class EESubsystem extends SubsystemBase {

    TalonSRX m_topBelt = new TalonSRX(4);
    TalonSRX m_bottomBelt = new TalonSRX(2);
    TalonSRX m_topShooter = new TalonSRX(3);
    TalonSRX m_bottomShooter = new TalonSRX(1);

    //TODO: Add a sensor to detect if we have a note
    private Boolean hasNote = true;

    public Boolean getHasNote() {
        return hasNote;
    }

    public EESubsystem() {
        m_topShooter.setInverted(true);
        m_bottomBelt.follow(m_topBelt);
    }

    public void intake() {
        m_topBelt.set(ControlMode.PercentOutput, 1);
    }

    public void stop() {
        m_topBelt.set(ControlMode.PercentOutput, 0);
        m_topShooter.set(ControlMode.PercentOutput, 0);
        m_bottomShooter.set(ControlMode.PercentOutput, 0);
    }

    public InstantCommand intakeCommand = new InstantCommand(() -> {
        while(!hasNote) {
            intake();
        }
    },
    this);

    public InstantCommand aimedShotCommand = new InstantCommand(() -> {
        //TODO: Aim and fire the shooter
    },
    this);
    
    public InstantCommand simpleShotCommand = new InstantCommand(() -> {
        //TODO: Fire the shooter
    },
    this);
}
