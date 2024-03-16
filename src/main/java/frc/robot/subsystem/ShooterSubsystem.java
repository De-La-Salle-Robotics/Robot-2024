package frc.robot.subsystem;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class ShooterSubsystem implements Subsystem {
    private final Measure<Velocity<Angle>> SpeedTolerance = RotationsPerSecond.of(20);

    TalonFX leftSide = new TalonFX(20, "Default Name");
    TalonFX rightSide = new TalonFX(21, "Default Name");

    VoltageOut leftRequest = new VoltageOut(0);
    VoltageOut rightRequest = new VoltageOut(0);

    VelocityTorqueCurrentFOC leftSpeedRequest = new VelocityTorqueCurrentFOC(0);
    VelocityTorqueCurrentFOC rightSpeedRequest = new VelocityTorqueCurrentFOC(0);

    public enum TargetSpeeds{
        Off(0, 0),
        PodiumShot(-26.65, -26.65),
        SubwooferShot(-26, -26),
        AmpShot(-40, -40);

        double LeftSpeed = 0;
        double RightSpeed = 0;

        private TargetSpeeds(double leftTarget, double rightTarget) {
            LeftSpeed = leftTarget;
            RightSpeed = rightTarget;
        }
    }

    public ShooterSubsystem(){
        var leftConfigs = new TalonFXConfiguration();
        leftConfigs.Slot0.withKP(5);
        leftSide.getConfigurator().apply(leftConfigs);

        var rightConfigs = new TalonFXConfiguration();
        rightConfigs.Slot0.withKP(5);
        rightSide.getConfigurator().apply(rightConfigs);
    }

    public Command manualCommand(DoubleSupplier output) {
        return run(()->{
            leftSide.setControl(leftRequest.withOutput(output.getAsDouble()));
            rightSide.setControl(rightRequest.withOutput(output.getAsDouble()));
        });
    }

    public Command goToSpeed(Supplier<TargetSpeeds> target) {
        return run(()-> {
            var targetEnum = target.get();
            leftSide.setControl(leftSpeedRequest.withVelocity(targetEnum.LeftSpeed));
            rightSide.setControl(rightSpeedRequest.withVelocity(targetEnum.RightSpeed));
        });
    }

    public boolean atSpeed() {
        return (leftSide.getClosedLoopError().getValue() < SpeedTolerance.in(RotationsPerSecond)) &&
               (rightSide.getClosedLoopError().getValue() < SpeedTolerance.in(RotationsPerSecond));
    }
}
