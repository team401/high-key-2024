package frc.robot.subsystems.scoring;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkLimitSwitch;

import frc.robot.constants.ConstantsLoader;
import frc.robot.constants.ConversionConstants;
import java.util.ArrayList;
import java.util.List;

public class ShooterIOTalonFX implements ShooterIO {
    private final CANSparkFlex kicker =
            new CANSparkFlex(ConstantsLoader.ScoringConstants.kickerMotorId, MotorType.kBrushless);

    private final TalonFX shooterLeft = new TalonFX(ConstantsLoader.ScoringConstants.shooterLeftMotorId);
    private final TalonFX shooterRight = new TalonFX(ConstantsLoader.ScoringConstants.shooterRightMotorId);

    private final Slot0Configs slot0 = new Slot0Configs();

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
                        .withStatorCurrentLimit(ConstantsLoader.ScoringConstants.shooterCurrentLimit)
                        .withStatorCurrentLimitEnable(true));

        TalonFXConfigurator shooterRightConfig = shooterRight.getConfigurator();
        shooterRightConfig.apply(
                new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(ConstantsLoader.ScoringConstants.shooterCurrentLimit)
                        .withStatorCurrentLimitEnable(true));

        slot0.withKP(ConstantsLoader.ScoringConstants.shooterkP);
        slot0.withKI(ConstantsLoader.ScoringConstants.shooterkI);
        slot0.withKD(ConstantsLoader.ScoringConstants.shooterkD);

        slot0.withKS(ConstantsLoader.ScoringConstants.shooterkS);
        slot0.withKV(ConstantsLoader.ScoringConstants.shooterkV);
        slot0.withKA(ConstantsLoader.ScoringConstants.shooterkA);

        shooterLeft.getConfigurator().apply(slot0);
        shooterRight.getConfigurator().apply(slot0);
    }

    @Override
    public void setShooterVelocityRPM(double velocity) {
        goalLeftVelocityRPM = velocity;
        goalRightVelocityRPM = velocity * ConstantsLoader.ScoringConstants.shooterOffsetAdjustment;
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

        inputs.kickerAppliedVolts = kicker.getBusVoltage();
        inputs.kickerStatorCurrentAmps = kicker.getOutputCurrent();

        inputs.noteInShooter =
                kicker.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen).isPressed();
    }

    @Override
    public void applyOutputs(ShooterOutputs outputs) {
        outputs.shooterLeftGoalVelocityRPM = goalLeftVelocityRPM;
        outputs.shooterRightGoalVelocityRPM = goalRightVelocityRPM;
        outputs.kickerGoalVolts = kickerVolts;

        if (override) {
            shooterLeft.setVoltage(overrideVolts);
            shooterRight.setVoltage(overrideVolts);
            return;
        }

        if (outputs.shooterLeftGoalVelocityRPM == 0.0) {
            shooterLeft.setVoltage(0.0);
            shooterRight.setVoltage(0.0);
        } else {
            shooterLeft.setControl(
                    new VelocityDutyCycle(
                            outputs.shooterLeftGoalVelocityRPM
                                    / ConversionConstants.kMinutesToSeconds));
            shooterRight.setControl(
                    new VelocityDutyCycle(
                            outputs.shooterRightGoalVelocityRPM
                                    / ConversionConstants.kMinutesToSeconds));
        }

        kicker.setVoltage(outputs.kickerGoalVolts);
    }

    @Override
    public List<TalonFX> getOrchestraMotors() {
        ArrayList<TalonFX> motors = new ArrayList<TalonFX>();
        motors.add(shooterLeft);
        motors.add(shooterRight);

        return motors;
    }
}
