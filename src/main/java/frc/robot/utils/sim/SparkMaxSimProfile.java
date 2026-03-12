package frc.robot.utils.sim;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.system.plant.DCMotor;

public class SparkMaxSimProfile extends SparkSimProfile {
    public SparkMaxSimProfile(final SparkMax spark, final DCMotor motor, final double rotorInertia) {
        super(new SparkMaxSim(spark, motor), motor, rotorInertia);
    }
}