// package frc.robot.subsystems.scoring;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.controller.ArmFeedforward;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.system.plant.DCMotor;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
// import frc.robot.constants.ScoringConstants;
// import frc.robot.constants.SimConstants;

// public class AimerIOSim implements AimerIO {
//     private final SingleJointedArmSim sim =
//             new SingleJointedArmSim(
//                     DCMotor.getKrakenX60(1),
//                     80,
//                     SingleJointedArmSim.estimateMOI(0.3872, 8.61),
//                     0.3872,
//                     0.0,
//                     2 * Math.PI,
//                     true,
//                     3 / 2 * Math.PI);
//     private final PIDController controller =
//             new PIDController(
//                     ScoringConstants.aimerkP, ScoringConstants.aimerkI, ScoringConstants.aimerkD);
//     private final ArmFeedforward feedforward =
//             new ArmFeedforward(
//                     ScoringConstants.aimerkS,
//                     ScoringConstants.aimerkG,
//                     ScoringConstants.aimerkV,
//                     ScoringConstants.aimerkA);
//     private final TrapezoidProfile profile =
//             new TrapezoidProfile(
//                     new TrapezoidProfile.Constraints(
//                             ScoringConstants.aimerCruiseVelocity,
//                             ScoringConstants.aimerAcceleration));

//     private final Timer timer = new Timer();

//     private boolean override = false;
//     private double overrideVolts = 0.0;

//     double previousGoalAngle = 0.0;

//     double minAngleClamp = 0.0;
//     double maxAngleClamp = 0.0;

//     double goalAngleRot = 0.0;
//     double controlSetpoint = 0.0;
//     double appliedVolts = 0.0;

//     double initialAngle = 0.0;
//     double initialVelocity = 0.0;

//     @Override
//     public void setAimAngleRot(double goalAngleRot) {
//         this.goalAngleRot = goalAngleRot;
//     }

//     @Override
//     public void controlAimAngleRot() {
//         if (goalAngleRot != previousGoalAngle) {
//             timer.reset();
//             timer.start();

//             initialAngle = Units.radiansToRotations(sim.getAngleRads());
//             initialVelocity = Units.radiansToRotations(sim.getVelocityRadPerSec());

//             previousGoalAngle = goalAngleRot;
//         }
//         goalAngleRot = MathUtil.clamp(goalAngleRot, minAngleClamp, maxAngleClamp);
//     }

//     @Override
//     public void setAngleClampsRot(double minAngleClamp, double maxAngleClamp) {
//         // TODO: Figure out why we needed this method, and figure out how to make it work
//         if (minAngleClamp > maxAngleClamp) {
//             return;
//         }
//         // These two lines look like they should make the minimum angle 0.0 rotations,
//         // which would mean that the arm could no longer go below horizontal.
//         this.minAngleClamp =
//                 MathUtil.clamp(minAngleClamp, 0.0, ScoringConstants.aimMaxAngleRotations);
//         this.maxAngleClamp =
//                 MathUtil.clamp(maxAngleClamp, 0.0, ScoringConstants.aimMaxAngleRotations);
//     }

//     @Override
//     public void setOverrideMode(boolean override) {
//         this.override = override;
//     }

//     @Override
//     public void setOverrideVolts(double volts) {
//         appliedVolts = volts;
//     }

//     public void setNegativeHomeLockMode(boolean lock) {}

//     @Override
//     public void setPID(double p, double i, double d) {
//         controller.setP(p);
//         controller.setI(i);
//         controller.setD(d);
//     }

//     @Override
//     public void updateInputs(AimerInputs inputs) {
//         sim.update(SimConstants.loopTime);

//         inputs.aimGoalAngleRot = goalAngleRot;
//         inputs.aimProfileGoalAngleRot = controlSetpoint;
//         inputs.aimAngleRot = Units.radiansToRotations(sim.getAngleRads());

//         inputs.aimVelocityRotPerSec = Units.radiansToRotations(sim.getVelocityRadPerSec());

//         inputs.aimStatorCurrentAmps = sim.getCurrentDrawAmps();
//     }

//     @Override
//     public void applyOutputs(AimerOutputs outputs) {
//         State trapezoidSetpoint =
//                 profile.calculate(
//                         timer.get(),
//                         new State(initialAngle, initialVelocity),
//                         new State(goalAngleRot, 0));
//         double controlSetpoint =
//                 MathUtil.clamp(
//                         trapezoidSetpoint.position,
//                         ScoringConstants.aimMinAngleRotations,
//                         ScoringConstants.aimMaxAngleRotations);
//         double velocitySetpoint = trapezoidSetpoint.velocity;

//         if (override) {
//             appliedVolts = overrideVolts;
//         } else {
//             appliedVolts =
//                     feedforward.calculate(controlSetpoint, velocitySetpoint)
//                             + controller.calculate(
//                                     Units.radiansToRotations(sim.getAngleRads()), controlSetpoint);
//             appliedVolts = MathUtil.clamp(appliedVolts, -12.0, 12.0);
//         }

//         outputs.aimAppliedVoltage = appliedVolts;

//         sim.setInputVoltage(outputs.aimAppliedVoltage);
//     }
// }
