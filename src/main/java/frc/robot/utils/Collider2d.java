package frc.robot.utils;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Collider2d {
    private Supplier<Pose2d> poseSupplier;

    public Collider2d(Supplier<Pose2d> robotPoseSupplier) {
        this.poseSupplier = robotPoseSupplier;
    }

    /**
     * Create a {@link Trigger} for when the robot enters a defined zone on the field
     * @param c1 the lower left corner of the trigger zone
     * @param c2 the upper right corner of the trigger zone
     * @return A {@link Trigger}
     */
    public Trigger rectangleCollider(Translation2d c1, Translation2d c2) {
        if(c1.getX() >= c2.getX() || c1.getY() >= c2.getY()) throw new IllegalArgumentException("c1 must be closer to 0 0 then c2");

        return new Trigger(() -> {
            Translation2d pos = poseSupplier.get().getTranslation();
            return pos.getX() > c1.getX() && pos.getX() <= c2.getX() && pos.getY() >= c1.getY() && pos.getY() <= c2.getY();
        });
    }

    /**
     * Create a {@link Trigger} for when the robot enters a defined zone on the field
     * @param center The center point of the collider
     * @param radius The radius of the collider
     * @return
     */
    public Trigger sphereCollider(Translation2d center, double radius) {
        return new Trigger(() -> poseSupplier.get().getTranslation().getDistance(center) < radius);
    }

    private double sign(Translation2d c1, Translation2d c2, Translation2d c3) {
        return (c1.getX() - c3.getX()) * (c2.getY() - c3.getY()) - (c2.getX() - c3.getX()) * (c1.getY() - c3.getY());
    }

    /**
     * Create a {@link Trigger} for when the robot enters a defined zone on the field
     * @param c1 one point of the triangle
     * @param c2 one point of the triangle
     * @param c3 one point of the triangle
     * @return A {@link Trigger}
     */
    public Trigger triangleCollider(Translation2d c1, Translation2d c2, Translation2d c3) {

        return new Trigger(() -> {
            Translation2d pos = poseSupplier.get().getTranslation();
            double d1, d2, d3;
            boolean hasNeg, hasPos;

            d1 = sign(pos, c1, c2);
            d2 = sign(pos, c2, c3);
            d3 = sign(pos, c3, c1);

            hasNeg = (d1 < 0) || (d2 < 0) || (d3 < 0);
            hasPos = (d1 > 0) || (d2 > 0) || (d3 > 0);

            return !(hasNeg && hasPos);
        });
    }

    public Trigger polygonCollider(Poly poly, boolean flipAlliance) {
        return new Trigger(() -> poly.contains(poseSupplier.get().getTranslation(), flipAlliance));
    }

    public Trigger polygonCollider(Poly poly) {
        return polygonCollider(poly, true);
    }

    public class Poly {
        private Translation2d[] vertices;
    
        public Poly(Translation2d... vertices) {
            this.vertices = vertices;
            if (this.vertices.length < 3) throw new IllegalArgumentException("Polygon must have minimum 3 points");
        }
    
        public boolean contains(Translation2d point, boolean flipAlliance) {
            int n = vertices.length;
            int i, j;
            boolean c = false;
            for (i = 0, j = n - 1; i < n; j = i++) {
                Translation2d vertexI = flipAlliance ? AllianceFlipUtil.apply(vertices[i]) : vertices[i];
                Translation2d vertexJ = flipAlliance ? AllianceFlipUtil.apply(vertices[j]) : vertices[j];
                if (((vertexI.getY() > point.getY()) != (vertexJ.getY() > point.getY())) &&
                        (point.getX() < (vertexJ.getX() - vertexI.getX()) * (point.getY() - vertexI.getY()) / (vertexJ.getY() - vertexI.getY()) + vertexI.getX())) {
                    c = !c;
                }
            }
            return c;
        }
    
        public Translation2d[] getVerticies(boolean allianceFlip) {
            Translation2d[] v = new Translation2d[vertices.length + 1];
            for (int i = 0; i < vertices.length; i++) {
                v[i] = allianceFlip ? AllianceFlipUtil.apply(vertices[i]) : vertices[i];
            }
            v[vertices.length] = v[0];
            return v;
        }
    }
}
