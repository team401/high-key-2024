package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.Constants.SensorConstants;

public class IntakeNEOVortex implements IntakeIO {

    private TalonFX leftIntake =
            new TalonFX(IntakeConstants.leftIntakeMotorID);
    private TalonFX rightIntake =
            new TalonFX(IntakeConstants.rightIntakeMotorID);

    private TalonFX belt = new TalonFX(IntakeConstants.indexTwoMotorID);

    DigitalInput bannerSensor = new DigitalInput(SensorConstants.uptakeSensorPort);

    private double goalVoltsLeftIntake, goalVoltsRightIntake, goalVoltsBelt;

    public IntakeNEOVortex() {
        TalonFXConfigurator leftIntakeConfig = leftIntake.getConfigurator();
        leftIntakeConfig.apply(
                new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(120)
                        .withStatorCurrentLimitEnable(true));

        TalonFXConfigurator rightIntakeConfig = rightIntake.getConfigurator();
        rightIntakeConfig.apply(
                new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(120)
                        .withStatorCurrentLimitEnable(true));

        leftIntake.setInverted(true);
        rightIntake.setInverted(true);

        belt.setInverted(true);

        TalonFXConfigurator beltConfig = belt.getConfigurator();
        beltConfig.apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
        beltConfig.apply(
                new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(60)
                        .withStatorCurrentLimitEnable(true));
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.leftIntakeGoalVoltage = goalVoltsLeftIntake;
        inputs.leftIntakeVoltage = leftIntake.getMotorVoltage().getValueAsDouble();
        inputs.leftIntakeStatorCurrent = leftIntake.getStatorCurrent().getValueAsDouble();

        inputs.rightIntakeGoalVoltage = goalVoltsRightIntake;
        inputs.rightIntakeVoltage = rightIntake.getMotorVoltage().getValueAsDouble();
        inputs.rightIntakeStatorCurrent = rightIntake.getStatorCurrent().getValueAsDouble();

        inputs.beltGoalVoltage = goalVoltsBelt;
        inputs.beltVoltage = belt.getMotorVoltage().getValueAsDouble();
        inputs.beltStatorCurrent = belt.getStatorCurrent().getValueAsDouble();
        inputs.beltSupplyCurrent = belt.getSupplyCurrent().getValueAsDouble();

        inputs.noteSensed = !bannerSensor.get();
    }

    @Override
    public void applyOutputs(IntakeIOInputs inputs) {
        leftIntake.set(inputs.leftIntakeGoalVoltage);
        rightIntake.set(inputs.rightIntakeGoalVoltage);
        belt.setControl(new VoltageOut(inputs.beltGoalVoltage));
    }
    
    @Override
    public void setIntakeVoltage(double volts) {
        goalVoltsRightIntake = volts / 12;
        goalVoltsLeftIntake = volts / 12;
    }

    @Override
    public void setBeltVoltage(double volts) {
        goalVoltsBelt = volts;
    }


}