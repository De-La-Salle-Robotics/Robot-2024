package frc.robot.subsystem;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class ArmSubsystem implements Subsystem {
    TalonFX leftSide = new TalonFX(10, "Default Name");
    TalonFX rightSide = new TalonFX(11, "Default Name");

    VoltageOut leftRequest = new VoltageOut(0);
    VoltageOut rightRequest = new VoltageOut(0);

    public ArmSubsystem(){}

    public Command manualCommand(DoubleSupplier output) {
        return new RunCommand(()->{
            leftSide.setControl(leftRequest.withOutput(output.getAsDouble()));
            rightSide.setControl(rightRequest.withOutput(-output.getAsDouble()));
        }, this);
    }
}
