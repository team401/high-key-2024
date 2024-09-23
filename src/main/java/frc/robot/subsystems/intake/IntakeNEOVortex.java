package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.constants.SensorConstants;
import frc.robot.constants.IntakeConstants;

public class IntakeNEOVortex implements IntakeIO {

    private CANSparkFlex intakeMotor =
            new CANSparkFlex(IntakeConstants.intakeMotorID, MotorType.kBrushless);
    private CANSparkFlex centeringMotor =
            new CANSparkFlex(IntakeConstants.centeringMotorID, MotorType.kBrushless);

    DigitalInput bannerSensor = new DigitalInput(SensorConstants.uptakeSensorPort);

    private double intakeMotorVolts, centeringMotorVolts;

    public IntakeNEOVortex() {
        intakeMotor.setSmartCurrentLimit(80);
        intakeMotor.setIdleMode(IdleMode.kBrake);

        intakeMotor.setInverted(true);

        centeringMotor.setSmartCurrentLimit(60);
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

        intakeMotor.setVoltage(outputs.intakeMotorVoltage);
        centeringMotor.setVoltage(outputs.centeringMotorVoltage);
    }

    @Override
    public void setIntakeVoltage(double volts) {
        intakeMotorVolts = volts;
        centeringMotorVolts = volts;
    }
}
