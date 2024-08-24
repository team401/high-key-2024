package frc.robot.subsystems.scoring;

import com.ctre.phoenix6.controls.VelocityDutyCycle;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;

public class ShooterIOSim implements ShooterIO {
    // TODO: Tune this later 
    private final FlywheelSim shooterLeftSim =
            new FlywheelSim(DCMotor.getKrakenX60(1), 0.5, 0.0010639061);
    private final FlywheelSim shooterRightSim =
            new FlywheelSim(DCMotor.getKrakenX60(1), 0.5, 0.0010639061);
        
    DigitalInput bannerSensor = new DigitalInput(Constants.SensorConstants.indexerSensorPort);

    double goalVelocityLeftRPM = 0.0;
    double goalVelocityRightRPM = 0.0;

    double kickerVolts = 0.0;

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.shooterLeftVelocityRPM = shooterLeftSim.getAngularVelocityRadPerSec();
        inputs.shooterLeftGoalVelocityRPM = goalVelocityLeftRPM;

        inputs.shooterRightVelocityRPM = shooterRightSim.getAngularVelocityRadPerSec();
        inputs.shooterRightGoalVelocityRPM = goalVelocityRightRPM;

        //inputs.fromIntakeMotorAppliedVolts = fromIntakeMotor.getMotorVoltage().getValueAsDouble();

    }


    @Override
    public void setShooterVelocity(double goalVelocity) {
        goalVelocityLeftRPM = goalVelocity;
        goalVelocityRightRPM = goalVelocity * ShooterConstants.offsetFlywheelVelocities;

        //shooterLeftSim.setInput(new VelocityDutyCycle(goalVelocityLeftRPM / 60));
        //shooterRightSim.setInput(new VelocityDutyCycle(goalVelocityLeftRPM / 60));
    }

    /*@Override
    public void setFromIntakeMotorVoltage(double kickerVoltage) {
        fromIntakeMotor.setVoltage(kickerVoltage);
    }

    @Override
    public void setAimAngleRad(double angle) {

    }*/


}