// TODO ADJUST CONSTANTS
// pendulum for scoring works very similar to other design but upside down

package frc.robot.subsystems.scoring;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
<<<<<<< HEAD
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
=======
>>>>>>> 3f9157c (formatting + removing second arm motor)
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
<<<<<<< HEAD
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.MathUtil;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;

=======
>>>>>>> 3f9157c (formatting + removing second arm motor)
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.ScoringConstants;
import org.littletonrobotics.junction.Logger;

public class AimerIORoboRio implements AimerIO {
<<<<<<< HEAD
    private final TalonFX aimerMotor = new TalonFX(ScoringConstants.aimerMotorId);
=======
    private final TalonFX aimerLeft = new TalonFX(ScoringConstants.aimMotorId);
>>>>>>> 3f9157c (formatting + removing second arm motor)

    private ControlRequest request;

<<<<<<< HEAD
    private final CANcoder aimerEncoder = new CANcoder(ScoringConstants.aimerEncoderId);
=======
    // replace with fusedencoder from aiden?
    // private final DutyCycleEncoder encoder = new
    // DutyCycleEncoder(ScoringConstants.aimEncoderPort);
>>>>>>> 3f9157c (formatting + removing second arm motor)

    private final Timer timer = new Timer();

    private boolean override = false;
    private double overrideVolts = 0.0;

    double previousGoalAngle = 0.0;

    double minAngleClamp = 0.0;
    double maxAngleClamp = 0.0;

    double goalAngleRad = 0.0;
    double appliedVolts = 0.0;

    double velocity = 0.0;

    double lastError = 0.0;
    double lastPosition = 0.0;
    double lastTime = Utils.getCurrentTimeSeconds();
    double controlSetpoint = 0.0;

    boolean motorDisabled = false;

    public AimerIORoboRio() {
        aimerMotor.setNeutralMode(NeutralModeValue.Brake);

        setStatorCurrentLimit(ScoringConstants.aimerCurrentLimit);

        CANcoderConfiguration cancoderConfiguration = new CANcoderConfiguration();
        // TODO: Check all of these values
        cancoderConfiguration.MagnetSensor.AbsoluteSensorRange =
                AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        cancoderConfiguration.MagnetSensor.SensorDirection =
                SensorDirectionValue.CounterClockwise_Positive;
        cancoderConfiguration.MagnetSensor.MagnetOffset = -ScoringConstants.aimerEncoderOffset;

        aimerEncoder.getConfigurator().apply(cancoderConfiguration);

        TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();

        talonFXConfigs.Feedback.FeedbackRemoteSensorID = aimerEncoder.getDeviceID();
        talonFXConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        talonFXConfigs.Feedback.SensorToMechanismRatio =
                ScoringConstants.aimerEncoderToMechanismRatio;
        talonFXConfigs.Feedback.RotorToSensorRatio = ScoringConstants.aimerRotorToSensorRatio;

        Slot0Configs slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.GravityType = GravityTypeValue.Arm_Cosine;

        slot0Configs.kS = ScoringConstants.aimerkS;
        slot0Configs.kV = ScoringConstants.aimerkV;
        slot0Configs.kA = ScoringConstants.aimerkA;
        slot0Configs.kG = ScoringConstants.aimerkG;

        slot0Configs.kP = ScoringConstants.aimerkP;
        slot0Configs.kI = ScoringConstants.aimerkI;
        slot0Configs.kD = ScoringConstants.aimerkD;

        MotionMagicConfigs motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = ScoringConstants.aimerCruiseVelocity;
        motionMagicConfigs.MotionMagicAcceleration = ScoringConstants.aimerAcceleration;

        aimerMotor.getConfigurator().apply(talonFXConfigs);
    }

    public void resetPID() {}

    @Override
    public void setAimAngleRad(double goalAngleRad) {
        this.goalAngleRad = goalAngleRad;
    }

    @Override
    public void controlAimAngleRad() {
        if (goalAngleRad != previousGoalAngle) {
            timer.reset();
            timer.start();

            previousGoalAngle = goalAngleRad;
        }
        goalAngleRad = MathUtil.clamp(goalAngleRad, minAngleClamp, maxAngleClamp);
    }

    @Override
    public void setAngleClampsRad(double minAngleClamp, double maxAngleClamp) {
        if (minAngleClamp > maxAngleClamp) {
            return;
        }
        this.minAngleClamp =
                MathUtil.clamp(
                        minAngleClamp,
                        ScoringConstants.aimMinAngleRadians,
                        ScoringConstants.aimMaxAngleRadians);
        this.maxAngleClamp =
                MathUtil.clamp(
                        maxAngleClamp,
                        ScoringConstants.aimMinAngleRadians,
                        ScoringConstants.aimMaxAngleRadians);
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
        Slot0Configs slot0Configs = new Slot0Configs();

        slot0Configs.kP = p;
        slot0Configs.kI = i;
        slot0Configs.kD = d;

        aimerMotor.getConfigurator().apply(slot0Configs);
    }

    @Override
    public void setMaxProfile(double maxVelocity, double maxAcceleration) {
        MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
        motionMagicConfigs.MotionMagicCruiseVelocity = maxVelocity;
        motionMagicConfigs.MotionMagicAcceleration = maxAcceleration;
        aimerMotor.getConfigurator().apply(motionMagicConfigs);
    }

    @Override
    public void setFF(double kS, double kV, double kA, double kG) {
        Slot0Configs slot0Configs = new Slot0Configs();

        slot0Configs.kS = kS;
        slot0Configs.kV = kV;
        slot0Configs.kA = kA;
        slot0Configs.kG = kG;

        aimerMotor.getConfigurator().apply(slot0Configs);
    }

    @Override
    public void setBrakeMode(boolean brake) {
<<<<<<< HEAD
        aimerMotor.setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }

    private double getEncoderPosition() {
        // return aimerRight.getPosition().getValueAsDouble() * 2.0 * Math.PI * (1.0 / 80.0);
        // return aimerEncoder.getAbsolutePosition().getValueAsDouble() * 2.0 * Math.PI -
        // ScoringConstants.aimerEncoderOffset;
        return aimerEncoder.getPosition().getValueAsDouble();
    }

    public void setStatorCurrentLimit(double limit) {
        TalonFXConfigurator aimerMotorConfig = aimerMotor.getConfigurator();
        aimerMotorConfig.apply(
=======
        aimerLeft.setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }

    private double getEncoderPosition() {
        return aimerLeft.getPosition().getValueAsDouble();
    }

    public void setStatorCurrentLimit(double limit) {
        TalonFXConfigurator aimerLeftConfig = aimerLeft.getConfigurator();
        aimerLeftConfig.apply(
>>>>>>> 3f9157c (formatting + removing second arm motor)
                new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(limit)
                        .withStatorCurrentLimitEnable(true));
    }

    @Override
    public void setMotorDisabled(boolean disabled) {
        motorDisabled = disabled;
    }

    @Override
    public void updateInputs(AimerInputs inputs) {
        // TODO: Add fault monitor for when encoder is unplugged
        // This value will probably not work for the current encoder to indicate unplugged status
        // if (getEncoderPosition() == -1.75) {
        //     motorDisabled = true;
        // }

        Logger.recordOutput("Scoring/motorDisabled", motorDisabled);

        inputs.aimGoalAngleRad = goalAngleRad;
        inputs.aimProfileGoalAngleRad = controlSetpoint;
        inputs.aimAngleRad = getEncoderPosition();

        double currentTime = Utils.getCurrentTimeSeconds();
        double diffTime = currentTime - lastTime;
        lastTime = currentTime;

        inputs.aimVelocityRadPerSec = (getEncoderPosition() - lastPosition) / diffTime;
        velocity = (getEncoderPosition() - lastPosition) / diffTime;
        lastPosition = getEncoderPosition();

        inputs.aimVelocityErrorRadPerSec =
                ((getEncoderPosition() - controlSetpoint) - lastError) / diffTime;
        lastError = getEncoderPosition() - controlSetpoint;

<<<<<<< HEAD
        inputs.aimStatorCurrentAmps = aimerMotor.getStatorCurrent().getValueAsDouble();
        inputs.aimSupplyCurrentAmps = aimerMotor.getSupplyCurrent().getValueAsDouble();
=======
        inputs.aimStatorCurrentAmps = aimerLeft.getStatorCurrent().getValueAsDouble();
        inputs.aimSupplyCurrentAmps = aimerLeft.getSupplyCurrent().getValueAsDouble();
>>>>>>> 3f9157c (formatting + removing second arm motor)
    }

    @Override
    public void applyOutputs(AimerOutputs outputs) {
        MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0.0);
        motionMagicVoltage.withPosition(goalAngleRad);

        request = motionMagicVoltage;

        controlSetpoint = aimerMotor.getClosedLoopReference().getValueAsDouble();
        if (override) {
            appliedVolts = overrideVolts;
            request = new VoltageOut(overrideVolts);
        } else {
            appliedVolts = aimerMotor.getClosedLoopOutput().getValueAsDouble();
        }

        if (!motorDisabled || override) {
<<<<<<< HEAD
            aimerMotor.setControl(request);
        } else {
            aimerMotor.setVoltage(0.0);
=======
            aimerLeft.setVoltage(outputs.aimAppliedVoltage);
        } else {
            aimerLeft.setVoltage(0.0);
>>>>>>> 3f9157c (formatting + removing second arm motor)
        }
    }
}
