package frc.robot.subsystem;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.AsynchronousInterrupt;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class IntakeSubsystem implements Subsystem {
    private final double IntakeVoltage = 4;
    private final double ShootVoltage = 3;

    TalonFX intakeMotor = new TalonFX(22, "Default Name");

    VoltageOut normalRequest = new VoltageOut(0);
    VoltageOut intakeRequest = new VoltageOut(0);
    DigitalInput noteSensor = new DigitalInput(0);
    AsynchronousInterrupt noteInterrupt = new AsynchronousInterrupt(noteSensor, this::noteCollectedInterrupt);

    public IntakeSubsystem(){
        intakeRequest.UpdateFreqHz = 0;

        var talonConfig = new TalonFXConfiguration();
        talonConfig.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);
        intakeMotor.getConfigurator().apply(talonConfig);
        
        noteInterrupt.setInterruptEdges(true, true);
        noteInterrupt.enable();
    }

    public Command manualCommand(DoubleSupplier output) {
        return run(()->{
            intakeMotor.setControl(normalRequest.withOutput(output.getAsDouble()));
        });
    }

    public Command intakeNote() {
        return run(()-> {
            intakeMotor.setControl(intakeRequest.withOutput(IntakeVoltage));
        }).until(this::hasNote);
    }

    public Command shootNote() {
        return run(()-> {
            intakeMotor.setControl(normalRequest.withOutput(ShootVoltage));
        });
    }

    public boolean hasNote() {
        return !noteSensor.get();
    }

    @Override
    public void periodic() {
        intakeRequest.LimitForwardMotion = hasNote();
    }

    private void noteCollectedInterrupt(boolean rising, boolean falling)
    {
        intakeRequest.LimitForwardMotion = falling;
        intakeMotor.setControl(intakeRequest);
    }
}
