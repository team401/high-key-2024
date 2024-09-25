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
<<<<<<< HEAD
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
=======
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.constants.IntakeConstants;
>>>>>>> 3b5f5b5 (attempted to run shooter hardware did not work can errors + talon issues)
import frc.robot.constants.SensorConstants;
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
    }

    @Override
    public void updateInputs(IntakeInputs inputs) {
        inputs.intakeMotorVoltage = intakeMotor.getAppliedOutput();
        inputs.intakeMotorStatorCurrentAmps = intakeMotor.getOutputCurrent();

<<<<<<< HEAD
        inputs.centeringMotorVoltage = centeringMotor.getAppliedOutput();
        inputs.centeringMotorStatorCurrentAmps = centeringMotor.getOutputCurrent();
=======
        inputs.rightIntakeVoltage = rightIntake.getBusVoltage();
        inputs.rightIntakeStatorCurrent = rightIntake.getOutputCurrent();

        inputs.beltVoltage = belt.getMotorVoltage().getValueAsDouble();
        inputs.beltStatorCurrent = belt.getStatorCurrent().getValueAsDouble();
        inputs.beltSupplyCurrent = belt.getSupplyCurrent().getValueAsDouble();

        inputs.noteSensed = false; // bannerSensor.get();
>>>>>>> 3b5f5b5 (attempted to run shooter hardware did not work can errors + talon issues)
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
