// package frc.robot.subsystems.scoring;

// import com.ctre.phoenix6.hardware.TalonFX;
// import java.util.ArrayList;
// import java.util.List;
// import org.littletonrobotics.junction.AutoLog;

// public interface AimerIO {
//     @AutoLog
//     public static class AimerInputs {
//         public double aimAngleRot = 0.0;
//         public double aimGoalAngleRot = 0.0;
//         public double aimProfileGoalAngleRot = 0.0;

//         public double aimStatorCurrentAmps = 0.0;
//         public double aimSupplyCurrentAmps = 0.0;

//         public double aimVelocityRotPerSec = 0.0;
//         public double aimVelocityErrorRotPerSec = 0.0;
//     }

//     @AutoLog
//     public static class AimerOutputs {
//         public double aimAppliedVoltage = 0.0;
//     }

//     public default void updateInputs(AimerInputs inputs) {}

//     public default void applyOutputs(AimerOutputs outputs) {}

//     public default void setAimAngleRot(double angle) {}

//     public default void controlAimAngleRot() {}

//     public default void setAngleClampsRot(double min, double max) {}

//     public default void setOverrideMode(boolean override) {}

//     public default void setOverrideVolts(double volts) {}

//     public default void setNegativeHomeLockMode(boolean lock) {}

//     public default void setPID(double p, double i, double d) {}

//     public default void resetPID() {}

//     public default void setMaxProfile(double maxVelocity, double maxAcceleration) {}

//     public default void setFF(double kS, double kV, double kA, double kG) {}

//     public default void setBrakeMode(boolean brake) {}

//     public default void setStatorCurrentLimit(double limit) {}

//     public default void setMotorDisabled(boolean disabled) {}

//     public default List<TalonFX> getOrchestraMotors() {
//         return new ArrayList<TalonFX>();
//     }
// }
