package frc.robot.utils.sim;

import com.revrobotics.spark.SparkSim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.utils.sim.PhysicsSim.SimProfile;

public class SparkSimProfile extends SimProfile {
    private static final double motorResistance = 0.002;

    protected final SparkSim sparkSim;
    protected final DCMotorSim motorSim;

    protected double nominalVbus = 12.0;

    public SparkSimProfile(final SparkSim sparkSim, final DCMotor motor, final double rotorInertia) {
        this.sparkSim = sparkSim;
        this.motorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(motor, rotorInertia, 1), motor);

        this.sparkSim.useDriverStationEnable();
    }

    @Override
    public void run() {
        final double dt = getPeriod();

        final double lastCurrent = motorSim.getCurrentDrawAmps();
        final double vbus = nominalVbus - lastCurrent * motorResistance;

        final double motorVoltage = sparkSim.getAppliedOutput() * vbus;

        motorSim.setInputVoltage(motorVoltage);
        motorSim.update(dt);

        final double rotPerSec = Units.radiansToRotations(motorSim.getAngularVelocityRadPerSec());
        final double rpm = rotPerSec * 60.0;

        sparkSim.iterate(rpm, vbus, dt);
    }
}