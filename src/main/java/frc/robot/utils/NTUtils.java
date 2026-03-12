package frc.robot.utils;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Robot;

public class NTUtils {
    public static void publishCommand(NetworkTable table, String key, Command cmd) {
        var cmdTable = table.getSubTable(key);
        cmdTable.getEntry(".type").setString("Command");
        cmdTable.getEntry(".name").setString(cmd.getName());
        cmdTable.getEntry(".controllable").setBoolean(true);
        NetworkTableEntry running = cmdTable.getEntry("running");
        running.setBoolean(false);
        Robot.getInstance().addPeriodic(() -> {
            boolean want = running.getBoolean(false);
            if (want && !cmd.isScheduled()) CommandScheduler.getInstance().schedule(cmd);
            else if (!want && cmd.isScheduled()) CommandScheduler.getInstance().cancel(cmd);
            running.setBoolean(cmd.isScheduled());
        }, 0.02);
    }
}
