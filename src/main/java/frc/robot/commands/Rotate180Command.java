package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Rotate180Command extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final Angle initialAngle;

    private final Supplier<Pose2d> currentPoseSupplier;

    private Angle errorAngle;

    private final DoublePublisher errorPub = NetworkTableInstance.getDefault()
            .getTable("rotate180")
            .getDoubleTopic("error")
            .publish();

    /** Network table publisher for the current robot heading. */
    private final DoublePublisher currentPub = NetworkTableInstance.getDefault()
            .getTable("rotate180")
            .getDoubleTopic("current")
            .publish();

    /** Network table publisher for the target heading angle. */
    private final DoublePublisher targetPub = NetworkTableInstance.getDefault()
            .getTable("rotate180")
            .getDoubleTopic("target")
            .publish();

    /**
     * Constructs a Rotate180Command.
     *
     * @param drivetrain the swerve drivetrain subsystem
     */
    public Rotate180Command(CommandSwerveDrivetrain drivetrain, Supplier<Pose2d> currentPoseSupplier) {
        this.drivetrain = drivetrain;
        this.currentPoseSupplier = currentPoseSupplier;
        this.initialAngle = currentPoseSupplier.get().getRotation().getMeasure();
        this.errorAngle = Degrees.of(0);

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        Pose2d currentPose = currentPoseSupplier.get();
        this.errorAngle = currentPose.getRotation().minus(new Rotation2d(initialAngle)).getMeasure();

        drivetrain.rotateToPose(currentPose, new Pose2d(currentPose.getX(), currentPose.getY(),
                new Rotation2d(initialAngle.minus(Degrees.of(180)))));

        errorPub.set(errorAngle.in(Degrees));
        currentPub.set(currentPose.getRotation().getDegrees());
        targetPub.set(initialAngle.in(Degrees));
    }

    /**
     * Finishes the command when the robot's heading matches the target angle
     * within the tolerance.
     *
     * @return {@code true} if the angular error is within tolerance
     */
    @Override
    public boolean isFinished() {
        return errorAngle.in(Degrees) < 1;
    }
}
