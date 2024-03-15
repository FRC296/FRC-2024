package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase{

    DoubleSolenoid climberLeft = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
    DoubleSolenoid climberRight = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);

    public void extendClimber() {
        climberLeft.set(Value.kForward);
        climberRight.set(Value.kForward);
    }

    public void retractClimber() {
        climberLeft.set(Value.kReverse);
        climberRight.set(Value.kReverse);
    }

    public InstantCommand extendClimberCommand = new InstantCommand(() -> {
        extendClimber();
    }, this);

    public InstantCommand retractClimberCommand = new InstantCommand(() -> {
        retractClimber();
    }, this);
}

