package frc.robot.utils.sim;

import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.sim.TalonFXSSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.utils.sim.PhysicsSim.SimProfile;

public class TalonFxsSimProfile extends SimProfile {
    private static final double motorResistance = 0.002; // Assume 2mOhm resistance for voltage drop calculation
    private final TalonFXSSimState sim;
    private final DCMotorSim motorSim;

    public TalonFxsSimProfile(final TalonFXS talon, final DCMotor motor, double rotorInertia) {
        this.motorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(motor, rotorInertia, 1), motor);
        this.sim = talon.getSimState();
    }

    /**
     * Runs the simulation profile.
     * 
     * This uses very rudimentary physics simulation and exists to allow users to
     * test features of our products in simulation using our examples out of the
     * box. Users may modify this to utilize more accurate physics simulation.
     */
    public void run() {
        /// DEVICE SPEED SIMULATION

        motorSim.setInputVoltage(sim.getMotorVoltage());

        motorSim.update(getPeriod());

        /// SET SIM PHYSICS INPUTS
        final double position_rot = motorSim.getAngularPositionRotations();
        final double velocity_rps = Units.radiansToRotations(motorSim.getAngularVelocityRadPerSec());

        sim.setRawRotorPosition(position_rot);
        sim.setRotorVelocity(velocity_rps);

        sim.setSupplyVoltage(12 - sim.getSupplyCurrent() * motorResistance);
    }
}
