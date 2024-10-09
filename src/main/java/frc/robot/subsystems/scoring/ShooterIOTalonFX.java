package frc.robot.subsystems.scoring;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.constants.ConversionConstants;
import frc.robot.constants.ScoringConstants;

public class ShooterIOTalonFX implements ShooterIO {
    // private final TalonFX kicker = new TalonFX(ScoringConstants.kickerMotorId);

    private final TalonFX shooterLeft = new TalonFX(ScoringConstants.shooterLeftMotorId);
    private final TalonFX shooterRight = new TalonFX(ScoringConstants.shooterRightMotorId);

    private final Slot0Configs slot0 = new Slot0Configs();

    /// DigitalInput bannerSensor = new DigitalInput(SensorConstants.indexerSensorPort);

    private boolean override = false;
    private double overrideVolts = 0.0;

    double goalLeftVelocityRPM = 0.0;
    double goalRightVelocityRPM = 0.0;

    double kickerVolts = 0.0;

    public ShooterIOTalonFX() {
        // kicker.setInverted(true);

        shooterLeft.setInverted(false);
        shooterRight.setInverted(true);

        shooterLeft.setNeutralMode(NeutralModeValue.Coast);
        shooterRight.setNeutralMode(NeutralModeValue.Coast);

        TalonFXConfigurator shooterLeftConfig = shooterLeft.getConfigurator();
        shooterLeftConfig.apply(
                new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(ScoringConstants.shooterCurrentLimit)
                        .withStatorCurrentLimitEnable(true));

        TalonFXConfigurator shooterRightConfig = shooterRight.getConfigurator();
        shooterRightConfig.apply(
                new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(ScoringConstants.shooterCurrentLimit)
                        .withStatorCurrentLimitEnable(true));

        // TalonFXConfigurator kickerConfig = kicker.getConfigurator();
        // kickerConfig.apply(
        //         new CurrentLimitsConfigs()
        //                 .withStatorCurrentLimit(ScoringConstants.kickerCurrentLimit)
        //                 .withStatorCurrentLimitEnable(true));

        slot0.withKP(ScoringConstants.shooterkP);
        slot0.withKI(ScoringConstants.shooterkI);
        slot0.withKD(ScoringConstants.shooterkD);

        slot0.withKS(ScoringConstants.shooterkS);
        slot0.withKV(ScoringConstants.shooterkV);
        slot0.withKA(ScoringConstants.shooterkA);

        shooterLeft.getConfigurator().apply(slot0);
        shooterRight.getConfigurator().apply(slot0);
    }

    @Override
    public void setShooterVelocityRPM(double velocity) {
        goalLeftVelocityRPM = velocity;
        goalRightVelocityRPM = velocity * ScoringConstants.shooterOffsetAdjustment;
    }

    @Override
    public void setKickerVolts(double volts) {
        kickerVolts = volts;
    }

    @Override
    public void setOverrideMode(boolean override) {
        this.override = override;
    }

    @Override
    public void setOverrideVolts(double volts) {
        overrideVolts = volts;
    }

    @Override
    public void setPID(double p, double i, double d) {
        slot0.withKP(p);
        slot0.withKI(i);
        slot0.withKD(d);

        shooterLeft.getConfigurator().apply(slot0);
        shooterRight.getConfigurator().apply(slot0);
    }

    @Override
    public void setFF(double kS, double kV, double kA) {
        slot0.withKS(kS);
        slot0.withKV(kV);
        slot0.withKA(kA);

        shooterLeft.getConfigurator().apply(slot0);
        shooterRight.getConfigurator().apply(slot0);
    }

    @Override
    public void updateInputs(ShooterInputs inputs) {

        inputs.shooterLeftVelocityRPM =
                shooterLeft.getVelocity().getValueAsDouble()
                        / ConversionConstants.kSecondsToMinutes;
        inputs.shooterLeftAppliedVolts = shooterLeft.getMotorVoltage().getValueAsDouble();
        inputs.shooterLeftStatorCurrentAmps = shooterLeft.getStatorCurrent().getValueAsDouble();
        inputs.shooterLeftSupplyCurrentAmps = shooterLeft.getSupplyCurrent().getValueAsDouble();

        inputs.shooterRightVelocityRPM =
                shooterRight.getVelocity().getValueAsDouble()
                        / ConversionConstants.kSecondsToMinutes;
        inputs.shooterRightAppliedVolts = shooterRight.getMotorVoltage().getValueAsDouble();
        inputs.shooterRightStatorCurrentAmps = shooterRight.getStatorCurrent().getValueAsDouble();
        inputs.shooterRightSupplyCurrentAmps = shooterRight.getSupplyCurrent().getValueAsDouble();

        // inputs.kickerAppliedVolts = kicker.getMotorVoltage().getValueAsDouble();
        // inputs.kickerStatorCurrentAmps = kicker.getStatorCurrent().getValueAsDouble();

        inputs.bannerSensor = true; // !bannerSensor.get();
    }

    @Override
    public void applyOutputs(ShooterOutputs outputs) {
        shooterLeft.setVoltage(0.0);
        shooterRight.setVoltage(0.0);
        // kicker.setVoltage(0.0);
        return;
        // outputs.shooterLeftGoalVelocityRPM = goalLeftVelocityRPM;
        // outputs.shooterRightGoalVelocityRPM = goalRightVelocityRPM;
        // outputs.kickerGoalVolts = kickerVolts;

        // if (override) {
        //     shooterLeft.setVoltage(overrideVolts);
        //     shooterRight.setVoltage(overrideVolts);
        //     return;
        // }

        // if (outputs.shooterLeftGoalVelocityRPM == 0.0) {
        //     shooterLeft.setVoltage(0.0);
        //     shooterRight.setVoltage(0.0);
        // } else {
        //     shooterLeft.setControl(
        //             new VelocityDutyCycle(
        //                     outputs.shooterLeftGoalVelocityRPM
        //                             / ConversionConstants.kMinutesToSeconds));
        //     shooterRight.setControl(
        //             new VelocityDutyCycle(
        //                     outputs.shooterRightGoalVelocityRPM
        //                             / ConversionConstants.kMinutesToSeconds));
        // }

        // kicker.setVoltage(outputs.kickerGoalVolts);
    }
}
