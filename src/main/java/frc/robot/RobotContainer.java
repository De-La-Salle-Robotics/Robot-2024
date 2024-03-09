// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystem.ArmSubsystem;
import frc.robot.subsystem.IntakeSubsystem;
import frc.robot.subsystem.ShooterSubsystem;
import frc.robot.subsystem.ArmSubsystem.ArmPositions;
import frc.robot.subsystem.ShooterSubsystem.TargetSpeeds;
import frc.robot.subsystem.ClimbSubsystem;

public class RobotContainer {
    private double MaxSpeed = 6; // 6 meters per second desired top speed
    private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final CommandXboxController driverJoystick = new CommandXboxController(0); // Driver joystick
    private final CommandXboxController operatorJoystick = new CommandXboxController(1); // Op joystick
    private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                                     // driving in open loop
    private final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final ArmSubsystem armSubsystem = new ArmSubsystem();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    private final ClimbSubsystem climbSubsystem = new ClimbSubsystem();

    private enum RobotShootState {
        Amp,
        Podium,
        Subwoofer
    }
    RobotShootState m_currentState = RobotShootState.Amp;

    private void configureBindings() {
        /* Configure named commands here */
        NamedCommands.registerCommand("ZeroArm", armSubsystem.zeroArm(false));
        NamedCommands.registerCommand("AimAndShoot",
            armSubsystem.goToPosition(()->ArmPositions.Subwoofer)
                .alongWith(shooterSubsystem.goToSpeed(()->TargetSpeeds.SubwooferShot)).repeatedly()
                .until(()->armSubsystem.atPosition() && shooterSubsystem.atSpeed())
            .andThen(intakeSubsystem.shootNote()));
        NamedCommands.registerCommand("StowArm", armSubsystem.goToPosition(()->ArmPositions.Stow));

        drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> drive.withVelocityX(-driverJoystick.getLeftY() * MaxSpeed) // Drive forward with // negative Y (forward)
                        .withVelocityY(-driverJoystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(-driverJoystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
                ));

        // driverJoystick.a().whileTrue(drivetrain.applyRequest(() -> brake));

        driverJoystick.leftBumper().onTrue(armSubsystem.goToPosition(() -> ArmPositions.Down))
                                   .whileTrue(intakeSubsystem.intakeNote());
                        //.andThen(armSubsystem.goToPosition(()->ArmPositions.Stow)));

        driverJoystick.y().onTrue(armSubsystem.goToPosition(()->ArmPositions.Stow));
        driverJoystick.x().onTrue(new InstantCommand(()->m_currentState = RobotShootState.Amp));
        driverJoystick.a().onTrue(new InstantCommand(()->m_currentState = RobotShootState.Subwoofer));
        driverJoystick.b().onTrue(new InstantCommand(()->m_currentState = RobotShootState.Podium));

        driverJoystick.leftTrigger().onTrue(armSubsystem.goToPosition(()->
            switch(m_currentState) {
                case Amp -> ArmPositions.Amp;
                case Podium -> ArmPositions.Podium;
                case Subwoofer -> ArmPositions.Subwoofer;
            }
        )).whileTrue(shooterSubsystem.goToSpeed(() ->
            switch(m_currentState) {
                case Amp -> TargetSpeeds.AmpShot;
                case Podium -> TargetSpeeds.PodiumShot;
                case Subwoofer -> TargetSpeeds.SubwooferShot;
            }
        ));

        driverJoystick.rightTrigger().and(shooterSubsystem::atSpeed).and(armSubsystem::atPosition).onTrue(intakeSubsystem.shootNote());

        // reset the field-centric heading on left bumper press
        driverJoystick.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));
        driverJoystick.povRight().onTrue(drivetrain.applyRequest(()->
                                robotCentricDrive.withVelocityX(-driverJoystick.getLeftY() * MaxSpeed)
                                .withVelocityY(-driverJoystick.getLeftX() * MaxSpeed)
                                .withRotationalRate(-driverJoystick.getRightX() * MaxAngularRate)).until(driverJoystick.povLeft()));
        drivetrain.registerTelemetry(logger::telemeterize);

        new Trigger(()->Math.abs(operatorJoystick.getLeftY()) > 0.1).whileTrue(
            armSubsystem.manualCommand(()->-operatorJoystick.getLeftY())
        );

        operatorJoystick.back().onTrue(armSubsystem.zeroArm(true));
        operatorJoystick.povUp().onTrue(armSubsystem.goToPosition(()->ArmPositions.Stow));
        operatorJoystick.povDown().onTrue(armSubsystem.goToPosition(()->ArmPositions.Subwoofer));
        operatorJoystick.povLeft().onTrue(armSubsystem.goToPosition(()->ArmPositions.Amp));
        operatorJoystick.povRight().onTrue(armSubsystem.goToPosition(()->ArmPositions.Podium));

        operatorJoystick.leftTrigger().whileTrue(intakeSubsystem.manualCommand(()->7));
        operatorJoystick.leftBumper().whileTrue(intakeSubsystem.manualCommand(()->-7));

        operatorJoystick.rightBumper().whileTrue(shooterSubsystem.manualCommand(()->3.8));
        operatorJoystick.rightTrigger().whileTrue(shooterSubsystem.manualCommand(()->-3.8));

        operatorJoystick.y().whileTrue(climbSubsystem.manualCommand(()->1, ()->0));
        operatorJoystick.a().whileTrue(climbSubsystem.manualCommand(()->-1, ()->0));
        operatorJoystick.x().whileTrue(climbSubsystem.manualCommand(()->0, ()->1));
        operatorJoystick.b().whileTrue(climbSubsystem.manualCommand(()->0, ()->-1));

        // Arm
        armSubsystem.setDefaultCommand(armSubsystem.holdPosition());

        // Intake
        intakeSubsystem.setDefaultCommand(intakeSubsystem.manualCommand(() -> 0));

        // Shooter
        shooterSubsystem.setDefaultCommand(shooterSubsystem.manualCommand(() -> 0));

        // Climb
        climbSubsystem.setDefaultCommand(climbSubsystem.manualCommand(()->0, ()->0));
    }

    public RobotContainer() {
        configureBindings();
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
