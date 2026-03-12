package frc.robot.vision.odometry;

import java.util.List;
import java.util.Optional;
import java.util.function.BooleanSupplier;

public interface OdometryCamera {
    public Optional<PoseEstimate> update();
    public void addDisableCondition(BooleanSupplier condition);
    public void addDisableConditions(List<BooleanSupplier> conditions);
}
