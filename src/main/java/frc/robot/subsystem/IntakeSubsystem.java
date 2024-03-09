package frc.robot.subsystem;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class IntakeSubsystem implements Subsystem {
    private final double IntakeVoltage = 2;
    private final double ShootVoltage = 2;

    TalonFX intakeMotor = new TalonFX(22, "Default Name");

    VoltageOut leftRequest = new VoltageOut(0);

    public IntakeSubsystem(){}

    public Command manualCommand(DoubleSupplier output) {
        return new RunCommand(()->{
            intakeMotor.setControl(leftRequest.withOutput(output.getAsDouble()));
        }, this);
    }

    public Command intakeNote() {
        return new RunCommand(()-> {
            intakeMotor.setControl(leftRequest.withOutput(IntakeVoltage));
        }, this).until(this::hasNote);
    }

    public Command shootNote() {
        return new RunCommand(()-> {
            intakeMotor.setControl(leftRequest.withOutput(ShootVoltage));
        }, this);
    }

    public boolean hasNote() {
        return false;
    }
}
