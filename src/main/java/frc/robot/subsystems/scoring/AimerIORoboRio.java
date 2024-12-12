// // TODO ADJUST CONSTANTS
// // pendulum for scoring works very similar to other design but upside down

// package frc.robot.subsystems.scoring;

// import com.ctre.phoenix6.Utils;
// import com.ctre.phoenix6.configs.CANcoderConfiguration;
// import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
// import com.ctre.phoenix6.configs.MotionMagicConfigs;
// import com.ctre.phoenix6.configs.Slot0Configs;
// import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.configs.TalonFXConfigurator;
// import com.ctre.phoenix6.controls.ControlRequest;
// import com.ctre.phoenix6.controls.MotionMagicVoltage;
// import com.ctre.phoenix6.controls.VoltageOut;
// import com.ctre.phoenix6.hardware.CANcoder;
// import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
// import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
// import com.ctre.phoenix6.signals.GravityTypeValue;
// import com.ctre.phoenix6.signals.NeutralModeValue;
// import com.ctre.phoenix6.signals.SensorDirectionValue;
// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.wpilibj.Timer;
// import frc.robot.constants.ScoringConstants;
// import java.util.ArrayList;
// import java.util.List;
// import org.littletonrobotics.junction.Logger;

// public class AimerIORoboRio implements AimerIO {
//     private final TalonFX aimerMotor = new TalonFX(ScoringConstants.aimerMotorId);

//     MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0.0);
//     private ControlRequest request;

//     private final CANcoder aimerEncoder = new CANcoder(ScoringConstants.aimerEncoderId);

//     private final Timer timer = new Timer();

//     private boolean override = false;
//     private double overrideVolts = 0.0;

//     double previousGoalAngle = 0.0;

//     double minAngleClamp = 0.0;
//     double maxAngleClamp = 0.0;

//     double goalAngleRot = 0.0;
//     double appliedVolts = 0.0;

//     double velocity = 0.0;

//     double lastError = 0.0;
//     double lastPosition = 0.0;
//     double lastTime = Utils.getCurrentTimeSeconds();
//     double controlSetpoint = 0.0;

//     boolean motorDisabled = false;
//     boolean lockNegativeAtHome = false;

//     TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();

//     public AimerIORoboRio() {
//         aimerMotor.setNeutralMode(NeutralModeValue.Coast);

//         setStatorCurrentLimit(ScoringConstants.aimerCurrentLimit);

//         CANcoderConfiguration cancoderConfiguration = new CANcoderConfiguration();
//         // TODO: Check all of these values
//         cancoderConfiguration.MagnetSensor.AbsoluteSensorRange =
//                 AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
//         cancoderConfiguration.MagnetSensor.SensorDirection =
//                 SensorDirectionValue.CounterClockwise_Positive;
//         cancoderConfiguration.MagnetSensor.MagnetOffset = ScoringConstants.aimerEncoderOffset;

//         aimerEncoder.getConfigurator().apply(cancoderConfiguration);

//         talonFXConfigs.Feedback.FeedbackRemoteSensorID = aimerEncoder.getDeviceID();
//         talonFXConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
//         talonFXConfigs.Feedback.SensorToMechanismRatio =
//                 ScoringConstants.aimerEncoderToMechanismRatio;
//         talonFXConfigs.Feedback.RotorToSensorRatio = ScoringConstants.aimerRotorToSensorRatio;
//         talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
//         talonFXConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
//         talonFXConfigs.CurrentLimits.StatorCurrentLimit = ScoringConstants.aimerCurrentLimit;

//         Slot0Configs slot0Configs = talonFXConfigs.Slot0;
//         slot0Configs.GravityType = GravityTypeValue.Arm_Cosine;

//         slot0Configs.kS = ScoringConstants.aimerkS;
//         slot0Configs.kV = ScoringConstants.aimerkV;
//         slot0Configs.kA = ScoringConstants.aimerkA;
//         slot0Configs.kG = ScoringConstants.aimerkG;

//         slot0Configs.kP = ScoringConstants.aimerkP;
//         slot0Configs.kI = ScoringConstants.aimerkI;
//         slot0Configs.kD = ScoringConstants.aimerkD;

//         MotionMagicConfigs motionMagicConfigs = talonFXConfigs.MotionMagic;
//         motionMagicConfigs.MotionMagicCruiseVelocity = ScoringConstants.aimerCruiseVelocity;
//         motionMagicConfigs.MotionMagicAcceleration = ScoringConstants.aimerAcceleration;

//         aimerMotor.getConfigurator().apply(talonFXConfigs);
//     }

//     public void resetPID() {}

//     @Override
//     public void setAimAngleRot(double goalAngleRot) {
//         setNegativeHomeLockMode(false);
//         this.goalAngleRot = goalAngleRot;
//     }

//     @Override
//     public void controlAimAngleRot() {
//         if (goalAngleRot != previousGoalAngle) {
//             timer.reset();
//             timer.start();

//             previousGoalAngle = goalAngleRot;
//         }
//         goalAngleRot = MathUtil.clamp(goalAngleRot, minAngleClamp, maxAngleClamp);
//     }

//     @Override
//     public void setAngleClampsRot(double minAngleClamp, double maxAngleClamp) {
//         if (minAngleClamp > maxAngleClamp) {
//             return;
//         }
//         this.minAngleClamp =
//                 MathUtil.clamp(
//                         minAngleClamp,
//                         ScoringConstants.aimMinAngleRotations,
//                         ScoringConstants.aimMaxAngleRotations);
//         this.maxAngleClamp =
//                 MathUtil.clamp(
//                         maxAngleClamp,
//                         ScoringConstants.aimMinAngleRotations,
//                         ScoringConstants.aimMaxAngleRotations);
//     }

//     @Override
//     public void setOverrideMode(boolean override) {
//         this.override = override;
//     }

//     @Override
//     public void setOverrideVolts(double volts) {
//         overrideVolts = volts;
//     }

//     @Override
//     public void setNegativeHomeLockMode(boolean lock) {
//         lockNegativeAtHome = lock;
//     }

//     @Override
//     public void setPID(double p, double i, double d) {
//         Slot0Configs slot0Configs = new Slot0Configs();

//         slot0Configs.kP = p;
//         slot0Configs.kI = i;
//         slot0Configs.kD = d;

//         aimerMotor.getConfigurator().apply(slot0Configs);
//     }

//     @Override
//     public void setMaxProfile(double maxVelocity, double maxAcceleration) {
//         MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
//         motionMagicConfigs.MotionMagicCruiseVelocity = maxVelocity;
//         motionMagicConfigs.MotionMagicAcceleration = maxAcceleration;
//         aimerMotor.getConfigurator().apply(motionMagicConfigs);
//     }

//     @Override
//     public void setFF(double kS, double kV, double kA, double kG) {
//         Slot0Configs slot0Configs = new Slot0Configs();

//         slot0Configs.kS = kS;
//         slot0Configs.kV = kV;
//         slot0Configs.kA = kA;
//         slot0Configs.kG = kG;

//         aimerMotor.getConfigurator().apply(slot0Configs);
//     }

//     @Override
//     public void setBrakeMode(boolean brake) {
//         aimerMotor.setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
//     }

//     private double getEncoderPosition() {
//         // return aimerRight.getPosition().getValueAsDouble() * 2.0 * Math.PI * (1.0 / 80.0);
//         // return aimerEncoder.getAbsolutePosition().getValueAsDouble() * 2.0 * Math.PI -
//         // ScoringConstants.aimerEncoderOffset;
//         return aimerEncoder.getPosition().getValueAsDouble();
//     }

//     public void setStatorCurrentLimit(double limit) {
//         TalonFXConfigurator aimerMotorConfig = aimerMotor.getConfigurator();
//         aimerMotorConfig.apply(
//                 new CurrentLimitsConfigs()
//                         .withStatorCurrentLimit(limit)
//                         .withStatorCurrentLimitEnable(true));
//     }

//     @Override
//     public void setMotorDisabled(boolean disabled) {
//         motorDisabled = disabled;
//     }

//     @Override
//     public void updateInputs(AimerInputs inputs) {
//         // TODO: Add fault monitor for when encoder is unplugged
//         // This value will probably not work for the current encoder to indicate unplugged status
//         // if (getEncoderPosition() == -1.75) {
//         //     motorDisabled = true;
//         // }

//         Logger.recordOutput("Scoring/motorDisabled", motorDisabled);

//         inputs.aimGoalAngleRot = goalAngleRot;
//         inputs.aimProfileGoalAngleRot = controlSetpoint;
//         inputs.aimAngleRot = getEncoderPosition();

//         double currentTime = Utils.getCurrentTimeSeconds();
//         double diffTime = currentTime - lastTime;
//         lastTime = currentTime;

//         inputs.aimVelocityRotPerSec = (getEncoderPosition() - lastPosition) / diffTime;
//         velocity = (getEncoderPosition() - lastPosition) / diffTime;
//         lastPosition = getEncoderPosition();

//         inputs.aimVelocityErrorRotPerSec =
//                 ((getEncoderPosition() - controlSetpoint) - lastError) / diffTime;
//         lastError = getEncoderPosition() - controlSetpoint;

//         inputs.aimStatorCurrentAmps = aimerMotor.getStatorCurrent().getValueAsDouble();
//         inputs.aimSupplyCurrentAmps = aimerMotor.getSupplyCurrent().getValueAsDouble();
//     }

//     @Override
//     public void applyOutputs(AimerOutputs outputs) {
//         MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0.0);
//         motionMagicVoltage.withPosition(goalAngleRot);

//         request = motionMagicVoltage;

//         if (lockNegativeAtHome
//                 && getEncoderPosition()
//                         < ScoringConstants.aimMinAngleRotations
//                                 + ScoringConstants.intakeAngleToleranceRotations
//                 && !override) {
//             // talonFXConfigs.CurrentLimits.StatorCurrentLimit =
//             //        ScoringConstants.aimerCurrentLimit / 5;
//             // aimerMotor.getConfigurator().apply(talonFXConfigs);
//             request = new VoltageOut(ScoringConstants.aimLockVoltage);
//         } else {
//             talonFXConfigs.CurrentLimits.StatorCurrentLimit = ScoringConstants.aimerCurrentLimit;
//             // aimerMotor.getConfigurator().apply(talonFXConfigs);
//             motionMagicVoltage.withPosition(goalAngleRot);
//             request = motionMagicVoltage;
//         }

//         controlSetpoint = aimerMotor.getClosedLoopReference().getValueAsDouble();
//         if (override) {
//             appliedVolts = overrideVolts;
//             request = new VoltageOut(overrideVolts);
//         } else {
//             appliedVolts = aimerMotor.getClosedLoopOutput().getValueAsDouble();
//         }

//         if (!motorDisabled || override) {
//             aimerMotor.setControl(request);
//         } else {
//             aimerMotor.setVoltage(0.0);
//         }
//     }

//     @Override
//     public List<TalonFX> getOrchestraMotors() {
//         ArrayList<TalonFX> motors = new ArrayList<TalonFX>();
//         motors.add(aimerMotor);

//         return motors;
//     }
// }
