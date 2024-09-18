// TODO ADJUST CONSTANTS
// pendulum for scoring works very similar to other design but upside down

package frc.robot.subsystems.scoring;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.ScoringConstants;
import org.littletonrobotics.junction.Logger;

public class AimerIORoboRio implements AimerIO {
    private final TalonFX aimerLeft = new TalonFX(ScoringConstants.aimLeftMotorId);
    private final TalonFX aimerRight = new TalonFX(ScoringConstants.aimRightMotorId);

    private final PIDController controller =
            new PIDController(
                    ScoringConstants.aimerkP, ScoringConstants.aimerkI, ScoringConstants.aimerkD);
    private ArmFeedforward feedforward =
            new ArmFeedforward(
                    ScoringConstants.aimerkS,
                    ScoringConstants.aimerkG,
                    ScoringConstants.aimerkV,
                    ScoringConstants.aimerkA);
    private TrapezoidProfile profile =
            new TrapezoidProfile(
                    new TrapezoidProfile.Constraints(
                            ScoringConstants.aimCruiseVelocity, ScoringConstants.aimAcceleration));

    private final DutyCycleEncoder encoder = new DutyCycleEncoder(ScoringConstants.aimEncoderPort);

    private final Timer timer = new Timer();

    private boolean override = false;
    private double overrideVolts = 0.0;

    boolean newProfile = false;
    double previousGoalAngle = 0.0;

    double minAngleClamp = 0.0;
    double maxAngleClamp = 0.0;

    double goalAngleRad = 0.0;
    double appliedVolts = 0.0;

    double initialAngle = 0.0;
    double initialVelocity = 0.0;

    double velocity = 0.0;

    double lastError = 0.0;
    double lastPosition = 0.0;
    double lastTime = Utils.getCurrentTimeSeconds();
    double controlSetpoint = 0.0;

    boolean motorDisabled = false;

    public AimerIORoboRio() {
        aimerLeft.setControl(new Follower(ScoringConstants.aimRightMotorId, true));

        aimerLeft.setNeutralMode(NeutralModeValue.Brake);
        aimerRight.setNeutralMode(NeutralModeValue.Brake);

        aimerRight.setInverted(false);

        setStatorCurrentLimit(ScoringConstants.aimerCurrentLimit);

        aimerRight.setPosition(0.0);

        controller.setTolerance(ScoringConstants.aimAngleTolerance);
    }

    public void resetPID() {
        controller.reset();
    }

    @Override
    public void setAimAngleRad(double goalAngleRad, boolean newProfile) {
        this.goalAngleRad = goalAngleRad;
        this.newProfile = newProfile;
    }

    @Override
    public void controlAimAngleRad() {
        if (goalAngleRad != previousGoalAngle && newProfile) {
            timer.reset();
            timer.start();

            initialAngle =
                    MathUtil.clamp(getEncoderPosition(), 0.0, ScoringConstants.aimMaxAngleRadians);
            initialVelocity = velocity;

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
        controller.setP(p);
        controller.setI(i);
        controller.setD(d);
    }

    @Override
    public void setMaxProfile(double maxVelocity, double maxAcceleration) {
        profile =
                new TrapezoidProfile(
                        new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));
    }

    @Override
    public void setFF(double kS, double kV, double kA, double kG) {
        feedforward = new ArmFeedforward(kS, kG, kV, kA);
    }

    @Override
    public void setBrakeMode(boolean brake) {
        aimerLeft.setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
        aimerRight.setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }

    private double getEncoderPosition() {
        // return aimerRight.getPosition().getValueAsDouble() * 2.0 * Math.PI * (1.0 / 80.0);
        return encoder.getAbsolutePosition() * 2.0 * Math.PI - ScoringConstants.aimerEncoderOffset;
    }

    public void setStatorCurrentLimit(double limit) {
        TalonFXConfigurator aimerLeftConfig = aimerLeft.getConfigurator();
        aimerLeftConfig.apply(
                new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(limit)
                        .withStatorCurrentLimitEnable(true));

        TalonFXConfigurator aimerRightConfig = aimerRight.getConfigurator();
        aimerRightConfig.apply(
                new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(limit)
                        .withStatorCurrentLimitEnable(true));
    }

    @Override
    public void setMotorDisabled(boolean disabled) {
        motorDisabled = disabled;
    }

    @Override
    public void updateInputs(AimerIOInputs inputs) {

        if (getEncoderPosition() == -1.75) {
            motorDisabled = true;
        }

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

        inputs.aimStatorCurrentAmps = aimerRight.getStatorCurrent().getValueAsDouble();
        inputs.aimSupplyCurrentAmps = aimerRight.getSupplyCurrent().getValueAsDouble();
    }

    @Override
    public void applyOutputs(AimerIOInputs inputs) {

        double appliedVolts = 0.0;

        State trapezoidSetpoint =
                profile.calculate(
                        timer.get(),
                        new State(initialAngle, initialVelocity),
                        new State(goalAngleRad, 0));

        controlSetpoint = MathUtil.clamp(trapezoidSetpoint.position, minAngleClamp, maxAngleClamp);
        double velocitySetpoint = trapezoidSetpoint.velocity;

        if (override) {
            appliedVolts = overrideVolts;
        } else {
            double controllerVolts = controller.calculate(getEncoderPosition(), controlSetpoint);
            appliedVolts =
                    feedforward.calculate(controlSetpoint, velocitySetpoint) + controllerVolts;
        }

        appliedVolts = MathUtil.clamp(appliedVolts, -12.0, 12.0);

        if (!motorDisabled || override) {
            aimerRight.setVoltage(appliedVolts);
        } else {
            aimerRight.setVoltage(0.0);
        }
    }
}
