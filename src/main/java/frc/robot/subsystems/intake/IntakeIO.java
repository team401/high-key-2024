package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

    @AutoLog
    public static class IntakeIOInputs {
        public double intakeMotorVoltage = 0.0;
        public double intakeMotorStatorCurrentAmps = 0.0;
        public double intakeMotorSupplyCurrentAmps = 0.0;

        public double centeringMotorVoltage = 0.0;
        public double centeringMotorStatorCurrentAmps = 0.0;
        public double centeringMotorSupplyCurrentAmps = 0.0;

        public boolean noteSensed = false;
    }

    @AutoLog
    public static class IntakeIOOutputs {
        public double intakeMotorVoltage = 0.0;

        public double centeringMotorVoltage = 0.0;
    }

    public default void updateInputs(IntakeIOInputs inputs) {}

    public default void applyOutputs(IntakeIOOutputs outputs) {}

    public default void setIntakeVoltage(double volts) {}

    public default void setBeltVoltage(double volts) {}
}
