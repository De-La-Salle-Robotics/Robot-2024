package frc.robot.subsystem;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class ClimbSubsystem implements Subsystem {
    TalonFX climbMotor = new TalonFX(9, "Default Name");

    VoltageOut leftRequest = new VoltageOut(0);

    public ClimbSubsystem(){}

    public Command manualCommand(DoubleSupplier output) {
        return new RunCommand(()->{
            climbMotor.setControl(leftRequest.withOutput(output.getAsDouble()));
        }, this);
    }
}
