package frc.robot.subsystems.scoring;

public interface ShooterIO {
    
    //@AutoLog
    public static class ShooterIOInputs {

        public double shooterLeftVelocityRPM = 0.0;
        public double shooterLeftGoalVelocityRPM = 0.0;

        public double shooterRightVelocityRPM = 0.0;
        public double shooterRightGoalVelocityRPM = 0.0;

        public double pendulumAngleRad = 0.0;
        public double pendulumGoalAngleRad = 0.0;

        public double fromIntakeMotorAppliedVolts = 0.0;

        public double hasNote = 0.0;
    }

    public default void updateInputs(ShooterIOInputs inputs) {};

    public default void setShooterVelocity(double goalVelocity) {};

    public default void setFromIntakeMotorVoltage(double kickerVoltage) {};

    public default void setAimAngleRad(double angle) {};

}
