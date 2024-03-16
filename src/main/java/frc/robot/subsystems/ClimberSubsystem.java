package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase{

    Compressor compressor = new Compressor(61,PneumaticsModuleType.CTREPCM);
    DoubleSolenoid climberLeft = new DoubleSolenoid(61,PneumaticsModuleType.CTREPCM, 0, 1);
    DoubleSolenoid climberRight = new DoubleSolenoid(61,PneumaticsModuleType.CTREPCM, 7, 6);

    public ClimberSubsystem(){
        compressor.enableDigital();
    }

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

