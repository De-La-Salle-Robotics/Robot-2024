// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystem.ArmSubsystem;
import frc.robot.subsystem.IntakeSubsystem;
import frc.robot.subsystem.ShooterSubsystem;
import frc.robot.subsystem.ClimbSubsystem;

public class RobotContainer {
  private double MaxSpeed = 6; // 6 meters per second desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController joystick = new CommandXboxController(0); // Driver joystick
  private final CommandXboxController op_joystick = new CommandXboxController(1); // Op joystick
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(MaxSpeed);

  private final ArmSubsystem armSubsystem = new ArmSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final ClimbSubsystem climbSubsystem = new ClimbSubsystem();

  private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

    joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    joystick.b().whileTrue(drivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    // reset the field-centric heading on left bumper press
    joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);

    // Arm
    op_joystick.y().whileTrue(armSubsystem.manualCommand(()->7));
    op_joystick.a().whileTrue(armSubsystem.manualCommand(()->-3));
    armSubsystem.setDefaultCommand(armSubsystem.manualCommand(()->0));

    // Intake 
    op_joystick.leftBumper().whileTrue(intakeSubsystem.manualCommand(()->7));
    op_joystick.rightBumper().whileTrue(intakeSubsystem.manualCommand(()->-7));
    intakeSubsystem.setDefaultCommand(intakeSubsystem.manualCommand(()->0));

   
    //Shooter
    op_joystick.leftTrigger().whileTrue(shooterSubsystem.manualCommand(()->3.8));
    op_joystick.rightTrigger().whileTrue(shooterSubsystem.manualCommand(()->-3.8));
    op_joystick.leftTrigger().whileTrue(intakeSubsystem.manualCommand(()->-9));
    op_joystick.rightTrigger().whileTrue(intakeSubsystem.manualCommand(()->9));
    shooterSubsystem.setDefaultCommand(shooterSubsystem.manualCommand(()->0));

     //Amp
    op_joystick.button(7).whileTrue(shooterSubsystem.manualCommand(()->10));
    op_joystick.button(8).whileTrue(shooterSubsystem.manualCommand(()->-10));
    shooterSubsystem.setDefaultCommand(shooterSubsystem.manualCommand(()->0)); 
    op_joystick.button(7).whileTrue(intakeSubsystem.manualCommand(()->-7));
    op_joystick.button(8).whileTrue(intakeSubsystem.manualCommand(()->7));
    intakeSubsystem.setDefaultCommand(intakeSubsystem.manualCommand(()->0));

    //flywheel spinup
    op_joystick.x().onTrue(shooterSubsystem.manualCommand(()->-6));
    op_joystick.b().onTrue(shooterSubsystem.manualCommand(()->0));
    shooterSubsystem.setDefaultCommand(shooterSubsystem.manualCommand(()->0));

    //Climb
    joystick.leftTrigger().whileTrue(climbSubsystem.manualCommand(()->1));
    climbSubsystem.setDefaultCommand(climbSubsystem.manualCommand(()->0));
  }

  
  
    
  public RobotContainer() {
    configureBindings();
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
