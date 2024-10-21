package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends SubsystemBase {

    private IntakeIO io;
    private IntakeInputsAutoLogged inputs = new IntakeInputsAutoLogged();
    private IntakeOutputsAutoLogged outputs = new IntakeOutputsAutoLogged();

    private State state = State.IDLE;

    private BooleanSupplier shooterHasNote = () -> false;
    private BooleanSupplier shooterAtIntakePosition = () -> false;

    private IntakeAction action = IntakeAction.NONE;

    private double intakeOverrideVolts = 0.0;
    private double beltOverrideVolts = 0.0;

    public IntakeSubsystem(IntakeIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.recordOutput("Intake/state", state);
        Logger.recordOutput("Intake/action", action);
        Logger.processInputs("Intake/inputs", inputs);
        Logger.processInputs("Intake/outputs", outputs);

        switch (state) {
            case IDLE:
                idle();
                break;
            case SEEKING:
                seeking();
                break;
            case REVERSING:
                reversing();
                break;
            case OVERRIDE:
                override();
                break;
        }

        io.applyOutputs(outputs);
        // Logger.processInputs("intakeOutputs", outputs);
    }

    public void setShooterHasNoteSupplier(BooleanSupplier shooterHasNote) {
        this.shooterHasNote = shooterHasNote;
    }

    public void setShooterAtIntakePosition(BooleanSupplier shooterAtIntakePosition) {
        this.shooterAtIntakePosition = shooterAtIntakePosition;
    }

    public boolean hasNote() {
        return inputs.noteSensed;
    }

    public void run(IntakeAction action) {
        this.action = action;
    }

    public void toggle() {
        if (action == IntakeAction.NONE) {
            action = IntakeAction.INTAKE;
        } else {
            action = IntakeAction.NONE;
        }
    }

    private void idle() {
        if (action == IntakeAction.INTAKE && !shooterHasNote.getAsBoolean()) {
            state = State.SEEKING;
        } else if (action == IntakeAction.REVERSE) {
            state = State.REVERSING;
        } else if (action == IntakeAction.OVERRIDE) {
            state = State.OVERRIDE;
        }

        io.setBeltVoltage(0);
        io.setIntakeVoltage(0);
    }

    private void seeking() {
        if (action != IntakeAction.INTAKE) {
            state = State.IDLE;
        }

        boolean noNotes = !inputs.noteSensed && !shooterHasNote.getAsBoolean();
        boolean readyToPassNoteToShooter =
                !shooterHasNote.getAsBoolean() && shooterAtIntakePosition.getAsBoolean();

        if (noNotes || readyToPassNoteToShooter) {
            io.setIntakeVoltage(-IntakeConstants.intakePower);
        } else {
            io.setIntakeVoltage(0.0);
        }
    }

    private void reversing() {
        if (action != IntakeAction.REVERSE) {
            state = State.IDLE;
        }

        io.setIntakeVoltage(IntakeConstants.intakePower);
        io.setBeltVoltage(IntakeConstants.beltPower);
    }

    private void override() {
        if (action != IntakeAction.OVERRIDE) {
            state = State.IDLE;
        }

        io.setIntakeVoltage(intakeOverrideVolts);
        io.setBeltVoltage(beltOverrideVolts);
    }

    public void setOverrideVolts(double intake, double belt) {
        intakeOverrideVolts = intake;
        beltOverrideVolts = belt;
    }

    private enum State {
        IDLE, // do nothing
        SEEKING, // run intake wheels until a note is taken in
        REVERSING, // whole intake backwards
        OVERRIDE
    }

    public enum IntakeAction {
        NONE, // do nothing
        INTAKE, // Try to intake a note if you don't have one
        REVERSE, // run backwards
        OVERRIDE
    }
}
