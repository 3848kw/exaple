package frc.robot.utils.sim;

import java.util.ArrayList;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.system.plant.DCMotor;

/**
 * Manages physics simulation for CTRE products.
 */
public class PhysicsSim {
    private static final PhysicsSim sim = new PhysicsSim();
    private final ArrayList<SimProfile> simProfiles = new ArrayList<SimProfile>();

    /**
     * Gets the robot simulator instance.
     */
    public static PhysicsSim getInstance() {
        return sim;
    }


    public void addTalon(final TalonFX talon, final DCMotor motorType, final double rotorInertia) {
        if (talon != null) {
            TalonFxSimProfile simFalcon = new TalonFxSimProfile(talon, motorType, rotorInertia);
            simProfiles.add(simFalcon);
        }
    }

    public void addTalon(final TalonFX talon, final DCMotor motorType) {
        addTalon(talon, motorType, 0.001);
    }

    public void addSparkMax(final SparkMax spark, final DCMotor motorType, final double rotorInertia) {
        if (spark != null) {
            SimProfile profile = new SparkMaxSimProfile(spark, motorType, rotorInertia);
            simProfiles.add(profile);
        }
    }

    public void addSparkMax(final SparkMax spark, final DCMotor motorType) {
        addSparkMax(spark, motorType, 0.001);
    }

    public void addSparkFlex(final SparkFlex spark, final DCMotor motorType, final double rotorInertia) {
        if (spark != null) {
            SimProfile profile = new SparkFlexSimProfile(spark, motorType, rotorInertia);
            simProfiles.add(profile);
        }
    }

    public void addSparkFlex(final SparkFlex spark, final DCMotor motorType) {
        addSparkFlex(spark, motorType, 0.001);
    }

    /**
     * Runs the simulator:
     * - enable the robot
     * - simulate sensors
     */
    public void run() {
        // Simulate devices
        for (SimProfile simProfile : simProfiles) {
            simProfile.run();
        }
    }

    /**
     * Holds information about a simulated device.
     */
    static class SimProfile {
        private double lastTime;
        private boolean running = false;

        /**
         * Runs the simulation profile.
         * Implemented by device-specific profiles.
         */
        public void run() {
        }

        /**
         * Returns the time since last call, in seconds.
         */
        protected double getPeriod() {
            // set the start time if not yet running
            if (!running) {
                lastTime = Utils.getCurrentTimeSeconds();
                running = true;
            }

            double now = Utils.getCurrentTimeSeconds();
            final double period = now - lastTime;
            lastTime = now;

            return period;
        }
    }
}
