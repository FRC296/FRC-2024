package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class EESubsystem extends SubsystemBase {

    TalonSRX m_topBelt = new TalonSRX(4);
    TalonSRX m_bottomBelt = new TalonSRX(2);
    TalonSRX m_topShooter = new TalonSRX(3);
    TalonSRX m_bottomShooter = new TalonSRX(1);

    //TODO: Add a sensor to detect if we have a note
    Boolean hasNote = true;

    public EESubsystem() {
        m_topShooter.setInverted(true);
        m_bottomBelt.follow(m_topBelt);
    }

    public void intake() {
        m_topBelt.set(ControlMode.PercentOutput, 1);
    }

    public void shoot() {
        // TODO spin up the shooter until fire is pressed
        hasNote = false;
    }

    public void stop() {
        m_topBelt.set(ControlMode.PercentOutput, 0);
        m_topShooter.set(ControlMode.PercentOutput, 0);
        m_bottomShooter.set(ControlMode.PercentOutput, 0);
    }

    public RunCommand intakeCommand = new RunCommand(() -> {
        while(!hasNote) {
            intake();
        }
    },
    this);

    public RunCommand aimedShotCommand = new RunCommand(() -> {
        //TODO: Aim and fire the shooter
    },
    this);
    
}
