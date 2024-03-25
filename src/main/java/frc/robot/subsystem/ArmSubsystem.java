package frc.robot.subsystem;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ArmSubsystem implements Subsystem {
    private final double StallCurrent = 8;
    private final double FindZeroVoltage = 0.5;
    private final double StallCurrentDebounceTime = 0.3; // seconds
    private final Measure<Angle> TargetAngleTolerance = Degrees.of(30);
    private final Measure<Angle> ReferenceAtPosition = Degrees.of(0.1);

    TalonFX leftSide = new TalonFX(10, "Default Name");
    TalonFX rightSide = new TalonFX(11, "Default Name");

    VoltageOut leftVoltageRequest = new VoltageOut(0);
    TorqueCurrentFOC zeroRequest = new TorqueCurrentFOC(0);

    MotionMagicVoltage leftPositionRequest = new MotionMagicVoltage(0).withEnableFOC(false);

    boolean hasZeroed = false;
    boolean hasLastSetPosition = false;
    boolean startZero = false;

    double holdPosition = 0;
    
    public enum ArmPositions {
        Down(0),
        Podium(20),
        Subwoofer(14),
        Amp(54),
        Stow(40);

        double position = 0;

        private ArmPositions(double target) {
            position = target;
        }
    }

    public ArmSubsystem(){
        var leftConfigs = new TalonFXConfiguration();
        leftConfigs.CurrentLimits.withStatorCurrentLimit(20)
                                 .withStatorCurrentLimitEnable(true);
        leftConfigs.Slot0.withKP(10)
                         .withKD(0.1)
                         .withKV(.12);

        leftConfigs.MotionMagic.withMotionMagicAcceleration(200)
                               .withMotionMagicCruiseVelocity(60);
        leftConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        leftConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        leftSide.getConfigurator().apply(leftConfigs);

        var rightConfigs = new TalonFXConfiguration();
        rightConfigs.CurrentLimits.withStatorCurrentLimit(20)
                                  .withStatorCurrentLimitEnable(true);
        rightConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        rightConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        rightSide.getConfigurator().apply(rightConfigs);

        rightSide.setControl(new Follower(leftSide.getDeviceID(), true));
    }

    public Command manualCommand(DoubleSupplier output) {
        return run(()->{
            leftSide.setControl(leftVoltageRequest.withOutput(output.getAsDouble()));
        });
    }

    public Command goToPosition(Supplier<ArmPositions> targetPosition) {
        return run(()-> {
            leftSide.setControl(leftPositionRequest.withPosition(targetPosition.get().position));
        });
    }


    public Command holdPosition() {
        return new FunctionalCommand(
           ()->holdPosition = leftSide.getPosition().getValue(),
           ()->{
                leftSide.setControl(leftPositionRequest.withPosition(holdPosition));
            },
           (end)->{},
           ()->false,
           this);
    }

    public Command zeroArm(boolean forceZero) {
        return new ConditionalCommand(
                runOnce(()->startZero=true).andThen(
                run(()-> {
                    leftSide.setControl(zeroRequest.withOutput(-StallCurrent * 1.5).withMaxAbsDutyCycle(FindZeroVoltage));
                })
                    .until(new Trigger(()->startZero && Math.abs(leftSide.getVelocity().getValue()) < 1.0 && leftSide.getStatorCurrent().getValue() > StallCurrent).debounce(StallCurrentDebounceTime)))
                    .andThen(runOnce(()->{leftSide.setPosition(0); rightSide.setPosition(0);}))
                    .andThen(runOnce(()->hasZeroed = true))
                    .andThen(runOnce(()->startZero=false)),
                new InstantCommand(),
                () -> forceZero || !hasZeroed);
    }

    public boolean atPosition() {
        return (leftSide.getClosedLoopError().getValue() < TargetAngleTolerance.in(Rotations)) &&
               (leftSide.getClosedLoopReference().getValue() - leftPositionRequest.Position < ReferenceAtPosition.in(Rotations));
    }
}
