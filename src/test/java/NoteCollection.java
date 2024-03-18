import static org.junit.jupiter.api.Assertions.*;

import frc.robot.subsystem.IntakeSubsystem;

import java.lang.reflect.Field;

import com.ctre.phoenix.unmanaged.Unmanaged;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj2.command.Command;

import org.junit.jupiter.api.*;

public class NoteCollection {
    IntakeSubsystem subsystem;
    TalonFX intakeMotor;
    TalonFXSimState intakeSim;
    DigitalInput noteSensor;
    DIOSim noteSim;

    @SuppressWarnings("PMD.SignatureDeclareThrowsException")
    @BeforeEach
    public void constructDevices() throws Exception {
        subsystem = new IntakeSubsystem();
        subsystem.setDefaultCommand(subsystem.manualCommand(()->0));

        Field intake = subsystem.getClass().getDeclaredField("intakeMotor");
        intake.setAccessible(true);
        intakeMotor = (TalonFX)intake.get(subsystem);

        Field sensor = subsystem.getClass().getDeclaredField("noteSensor");
        sensor.setAccessible(true);
        noteSensor = (DigitalInput)sensor.get(subsystem);

        intakeSim = intakeMotor.getSimState();
        noteSim = new DIOSim(noteSensor);
        noteSim.setIsInput(true);
    }

    @Test
    public void testNote() {
        noteSim.setValue(true);
        subsystem.periodic();
        subsystem.simulationPeriodic();

        assertFalse(subsystem.hasNote());

        Command intakeCommand = subsystem.intakeNote();

        intakeCommand.initialize();

        waitForUpdate(0.05, intakeCommand);

        System.out.println("Intaking at " + intakeSim.getMotorVoltage() + " V");
        assertTrue(intakeSim.getMotorVoltage() > 2);

        noteSim.setValue(false);

        waitForUpdate(0.01, intakeCommand);
        /* It seems like simulation async interrupts aren't very fast, so rely on periodic to update */
        System.out.println("Caught note " + intakeSim.getMotorVoltage() + " V");
        assertTrue(intakeSim.getMotorVoltage() == 0);
    }

    private void waitForUpdate(double seconds, Command... commands) {
        try {
            for (int i = 0; i < 10; ++i) {
                Unmanaged.feedEnable(100);
                for (Command c : commands) {
                    c.execute();
                }
                subsystem.periodic();
                Thread.sleep((long) (seconds * 100));
            }
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
}