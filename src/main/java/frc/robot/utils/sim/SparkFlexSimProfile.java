package frc.robot.utils.sim;

import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.math.system.plant.DCMotor;

public class SparkFlexSimProfile extends SparkSimProfile {
    public SparkFlexSimProfile(final SparkFlex spark, final DCMotor motor, final double rotorInertia) {
        super(new SparkFlexSim(spark, motor), motor, rotorInertia);
    }
}
