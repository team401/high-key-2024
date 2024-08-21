package frc.robot.subsystems.scoring;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.SensorConstants;
import frc.robot.Constants.ShooterConstants;


public class ShooterIOTalon implements ShooterIO {
    
    //intake motors
    private final TalonFX fromIntakeMotor = new TalonFX(ShooterConstants.fromIntakeMotorID);
    private final TalonFX pendulumMotor = new TalonFX(ShooterConstants.pendulumMotorID);
    private final TalonFX leftFlywheel = new TalonFX(ShooterConstants.leftShooterMotorID);
    private final TalonFX rightFlywheel = new TalonFX(ShooterConstants.rightShooterMotorID);

    private final DigitalInput bannerSensor = new DigitalInput(SensorConstants.indexerSensorPort);

    private double goalVelocityLeftRPM;
    private double goalVelocityRightRPM;

    public ShooterIOTalon() {
        
        /*TODO: determine motor orientations
        kicker.setInverted(true);

        shooterLeft.setInverted(true);
        shooterRight.setInverted(false);*/

        leftFlywheel.setNeutralMode(NeutralModeValue.Coast);
        rightFlywheel.setNeutralMode(NeutralModeValue.Coast);

        TalonFXConfigurator fromIntConfig = fromIntakeMotor.getConfigurator();
        fromIntConfig.apply(
                new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(120)
                        .withStatorCurrentLimitEnable(true));

        TalonFXConfigurator pendConfigurator = pendulumMotor.getConfigurator();
        pendConfigurator.apply(
                new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(120)
                        .withStatorCurrentLimitEnable(true));

        TalonFXConfigurator leftFlywheelConfigurator = leftFlywheel.getConfigurator();
        leftFlywheelConfigurator.apply(
                new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(120)
                        .withStatorCurrentLimitEnable(true));

        TalonFXConfigurator rightFlywheelConfigurator = rightFlywheel.getConfigurator();
        rightFlywheelConfigurator.apply(
                new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(120)
                        .withStatorCurrentLimitEnable(true));
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.shooterLeftVelocityRPM = leftFlywheel.getVelocity().getValueAsDouble() * 60;
        inputs.shooterLeftGoalVelocityRPM = goalVelocityLeftRPM;

        inputs.shooterRightVelocityRPM = rightFlywheel.getVelocity().getValueAsDouble() * 60;
        inputs.shooterRightGoalVelocityRPM = goalVelocityRightRPM;

        inputs.fromIntakeMotorAppliedVolts = fromIntakeMotor.getMotorVoltage().getValueAsDouble();

    }

    //TODO: ask if the two flywheels will be shooting at different speeds
    @Override
    public void setShooterVelocity(double goalVelocity) {
        goalVelocityLeftRPM = goalVelocity;
        goalVelocityRightRPM = goalVelocity * ShooterConstants.offsetFlywheelVelocities;

        leftFlywheel.setControl(new VelocityDutyCycle(goalVelocityLeftRPM / 60));
        rightFlywheel.setControl(new VelocityDutyCycle(goalVelocityLeftRPM / 60));
    }

    @Override
    public void setFromIntakeMotorVoltage(double kickerVoltage) {
        fromIntakeMotor.setVoltage(kickerVoltage);
    }

    @Override
    public void setAimAngleRad(double angle) {

    }

}
