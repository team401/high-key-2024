package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.constants.IntakeConstants;

public class IntakeNEOVortex implements IntakeIO {

    private CANSparkFlex intakeMotor =
            new CANSparkFlex(IntakeConstants.intakeMotorID, MotorType.kBrushless);
    private CANSparkFlex centeringMotor =
            new CANSparkFlex(IntakeConstants.centeringMotorID, MotorType.kBrushless);

    // DigitalInput bannerSensor = new DigitalInput(SensorConstants.uptakeSensorPort);

    private double intakeMotorVolts, centeringMotorVolts;

    public IntakeNEOVortex() {
        intakeMotor.setSmartCurrentLimit(80);
        intakeMotor.setIdleMode(IdleMode.kBrake);

        intakeMotor.setInverted(true);

        centeringMotor.setSmartCurrentLimit(60);
        centeringMotor.setInverted(true);
    }

    @Override
    public void updateInputs(IntakeInputs inputs) {
        inputs.intakeMotorVoltage = intakeMotor.getAppliedOutput();
        inputs.intakeMotorStatorCurrentAmps = intakeMotor.getOutputCurrent();

        inputs.centeringMotorVoltage = centeringMotor.getAppliedOutput();
        inputs.centeringMotorStatorCurrentAmps = centeringMotor.getOutputCurrent();
    }

    @Override
    public void applyOutputs(IntakeOutputs outputs) {
        outputs.intakeMotorVoltage = intakeMotorVolts;

        outputs.centeringMotorVoltage = centeringMotorVolts;
        intakeMotor.setVoltage(intakeMotorVolts);
        centeringMotor.setVoltage(centeringMotorVolts);
    }

    @Override
    public void setIntakeVoltage(double volts) {
        intakeMotorVolts = volts;
        centeringMotorVolts = volts;
    }
}
