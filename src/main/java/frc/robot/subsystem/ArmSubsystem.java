package frc.robot.subsystem;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ArmSubsystem implements Subsystem {
    private final double StallCurrent = 10;
    private final double FindZeroVoltage = -0.4;
    private final double StallCurrentDebounceTime = 0.3; // seconds
    private final Measure<Angle> TargetAngleTolerance = Degrees.of(10);
    private final Measure<Angle> ReferenceAtPosition = Degrees.of(0.1);

    TalonFX leftSide = new TalonFX(10, "Default Name");
    TalonFX rightSide = new TalonFX(11, "Default Name");

    VoltageOut leftVoltageRequest = new VoltageOut(0);

    MotionMagicVoltage leftPositionRequest = new MotionMagicVoltage(0);

    boolean hasZeroed = false;
    boolean hasLastSetPosition = false;

    double holdPosition = 0;

    public enum ArmPositions {
        Down(0),
        Podium(1),
        Subwoofer(2),
        Amp(3),
        Stow(3);

        double position = 0;

        private ArmPositions(double target) {
            position = target;
        }
    }

    public ArmSubsystem(){
        var leftConfigs = new TalonFXConfiguration();
        leftConfigs.CurrentLimits.withStatorCurrentLimit(20)
                                 .withStatorCurrentLimitEnable(true);
        leftConfigs.Slot0.kP = 0;
        leftSide.getConfigurator().apply(leftConfigs);

        var rightConfigs = new TalonFXConfiguration();
        rightConfigs.CurrentLimits.withStatorCurrentLimit(20)
                                  .withStatorCurrentLimitEnable(true);
        rightSide.getConfigurator().apply(rightConfigs);

        rightSide.setControl(new Follower(leftSide.getDeviceID(), true));
    }

    public Command manualCommand(DoubleSupplier output) {
        return new RunCommand(()->{
            leftSide.setControl(leftVoltageRequest.withOutput(output.getAsDouble()));
        }, this);
    }

    public Command goToPosition(Supplier<ArmPositions> targetPosition) {
        return new RunCommand(()-> {
            leftSide.setControl(leftPositionRequest.withPosition(targetPosition.get().position));
        }, this);
    }


    public Command holdPosition() {
        return new FunctionalCommand(
           ()->holdPosition = leftSide.getPosition().getValue(),
           ()->leftSide.setControl(leftPositionRequest.withPosition(holdPosition)),
           (end)->{},
           ()->false,
           this);
    }

    public Command zeroArm(boolean forceZero) {
        return new ConditionalCommand(
                new RunCommand(()-> leftSide.setControl(leftVoltageRequest.withOutput(FindZeroVoltage)), this)
                    .until(new Trigger(()->leftSide.getStatorCurrent().getValue() > StallCurrent).debounce(StallCurrentDebounceTime))
                    .andThen(new InstantCommand(()->leftSide.setPosition(0), this))
                    .andThen(new InstantCommand(()->hasZeroed = true)),
                new InstantCommand(),
                () -> forceZero || !hasZeroed);
    }

    public boolean atPosition() {
        return (leftSide.getClosedLoopError().getValue() < TargetAngleTolerance.in(Rotations)) &&
               (leftSide.getClosedLoopReference().getValue() - leftPositionRequest.Position < ReferenceAtPosition.in(Rotations));
    }
}
