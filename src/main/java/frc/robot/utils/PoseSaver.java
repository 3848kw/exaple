package frc.robot.utils;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Scanner;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class PoseSaver {
    public static final PoseSaver instance = new PoseSaver();
    static File poseFile;

    public PoseSaver() {
        poseFile = new File("/home/lvuser/posefile");
        try {
            if(!poseFile.exists()) {
                poseFile.createNewFile();
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
        
    }

    /**
     * Write a pose to the RoboRio's PoseFile.
     * Note that only one PoseFile can exist at a time writing a new pose will overwrite the last PoseFile
     * @param pose The Pose saved in the PoseFile
     */
    public static void WritePose(Pose2d pose) {
        try(FileWriter writer = new FileWriter(poseFile)) {
            writer.append(String.valueOf(pose.getX()));
            writer.append(System.lineSeparator());
            writer.append(String.valueOf(pose.getY()));
            writer.append(System.lineSeparator());
            writer.append(String.valueOf(pose.getRotation().getDegrees()));
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    /**
     * Returns the pose in the RoboRio's PoseFile.
     * Returns null if the PoseFIle does not exist
     * @return Pose2d
     */
    public static Pose2d readPose() {
        if(!poseFile.exists()) {
            return null;
        }
        
        try(Scanner scanner = new Scanner(poseFile)) {
            String data = "";
            while (scanner.hasNextLine()) {
                data += scanner.nextLine() + System.lineSeparator();
                System.out.println(data);
            }
            String[] poseData = data.split(System.lineSeparator());
            scanner.close();
            if(poseData.length != 3) return null;
            return new Pose2d(Double.parseDouble(poseData[0]), Double.parseDouble(poseData[1]), Rotation2d.fromDegrees(Double.parseDouble(poseData[2])));
        } catch(IOException e) {
            e.printStackTrace();
            return null;
        }
    }
}
