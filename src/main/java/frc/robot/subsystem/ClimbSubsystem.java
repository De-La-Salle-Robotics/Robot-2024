package frc.robot.subsystem;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class ClimbSubsystem implements Subsystem {
    TalonFX climbMotor = new TalonFX(9, "Default Name");

    VictorSPX winchMotor = new VictorSPX(25);

    VoltageOut climbRequest = new VoltageOut(0);

    public ClimbSubsystem() {
        var climbConfig = new TalonFXConfiguration();
        climbConfig.CurrentLimits.withStatorCurrentLimit(50)
                                 .withStatorCurrentLimitEnable(true);
        climbMotor.getConfigurator().apply(climbConfig);

        winchMotor.configFactoryDefault();
    }

    public Command manualCommand(DoubleSupplier climbOutput, DoubleSupplier winchOutput) {
        return run(() -> {
            climbMotor.setControl(climbRequest.withOutput(climbOutput.getAsDouble()));

            winchMotor.set(ControlMode.PercentOutput, winchOutput.getAsDouble());

        });
    }
}
