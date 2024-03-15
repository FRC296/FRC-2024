package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase{

    Solenoid climberLeft = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
    Solenoid climberRight = new Solenoid(PneumaticsModuleType.CTREPCM, 1);

    public void extendClimber() {
        climberLeft.set(true);
        climberRight.set(true);
    }

    public void retractClimber() {
        climberLeft.set(false);
        climberRight.set(false);
    }

    public InstantCommand extendClimberCommand = new InstantCommand(() -> {
        extendClimber();
    }, this);

    public InstantCommand retractClimberCommand = new InstantCommand(() -> {
        retractClimber();
    }, this);
}

