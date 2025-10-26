
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class HolonomicPose {
    private final Pose2d pose;
    private final Rotation2d heading; // Direction of travel

    /**
     * Defines a Pose2d with heading, useful for swerve drives where direction of travel
     * is not necessarily the direction the robot is facing
     * @param pose Pose2d of final position, with rotation indicating direction of travel
     * @param heading Heading of final position, indicating direction the robot should be facing
     */
    public HolonomicPose(Pose2d pose, Rotation2d heading) {
        this.pose = pose;
        this.heading = heading;
    }

    public HolonomicPose(Translation2d translation, Rotation2d rotation, Rotation2d heading) {
        this.pose = new Pose2d(translation, rotation);
        this.heading = heading;
    }

    public Pose2d getPose() {
        return pose;
    }

    public Translation2d getTranslation() {
        return pose.getTranslation();
    }

    public Rotation2d getRotation() {
        return pose.getRotation();
    }

    public Rotation2d getHeading() {
        return heading;
    }

    @Override
    public String toString() {
        return String.format("PathPose(x=%.2f, y=%.2f, rot=%.2f°, heading=%.2f°)",
            pose.getX(), pose.getY(), pose.getRotation().getDegrees(), heading.getDegrees());
    }
}
