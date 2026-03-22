package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.Constants.GameConstants;

/**
 * Rotates the robot to face the hub/goal.
 * 
 * <p>
 * This command uses the robot's current position and alliance color to
 * determine
 * the target hub location, calculates the angle to face that hub, and commands
 * the
 * drivetrain to rotate accordingly while maintaining position. The command
 * finishes
 * when the robot's heading matches the target angle.
 * 
 * <p>
 * Requires: {@link CommandSwerveDrivetrain}
 */
public class RotateToHubCommand extends Command {
    private final CommandSwerveDrivetrain drivetrain;

    private final Supplier<Pose2d> currentPoseSupplier;

    /** The maximum allowable error in degrees. */
    private final static double kToleranceDegrees = 0.5;
    /** The current error between the robot's heading and the target angle. */
    private Angle errorAngle;

    /** Network table publisher for the angular error. */
    private final DoublePublisher errorPub = NetworkTableInstance.getDefault()
            .getTable("RotateToHub")
            .getDoubleTopic("Error")
            .publish();

    /** Network table publisher for the current robot heading. */
    private final DoublePublisher currentPub = NetworkTableInstance.getDefault()
            .getTable("RotateToHub")
            .getDoubleTopic("Current Angle")
            .publish();

    /** Network table publisher for the target heading angle. */
    private final DoublePublisher targetPub = NetworkTableInstance.getDefault()
            .getTable("RotateToHub")
            .getDoubleTopic("Target Angle")
            .publish();

    // dont change after init
    private Pose2d targetPose;

    /**
     * Constructs a RotateToHubCommand.
     *
     * @param drivetrain the swerve drivetrain subsystem
     */
    public RotateToHubCommand(CommandSwerveDrivetrain drivetrain, Supplier<Pose2d> currentPoseSupplier) {
        this.drivetrain = drivetrain;
        this.currentPoseSupplier = currentPoseSupplier;
        this.errorAngle = Degrees.of(0);

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        // calculate target
        Pose2d currentPose = currentPoseSupplier.get();
        Translation2d targetHub = (DriverStation.getAlliance().get() == Alliance.Blue) ? GameConstants.blueHubLocation
                : GameConstants.redHubLocation;

        Translation2d vectorToTarget = targetHub.minus(currentPose.getTranslation());

        targetPose = new Pose2d(currentPose.getTranslation(), vectorToTarget.getAngle());

        targetPub.set(targetPose.getRotation().getDegrees());
    }

    @Override
    public void execute() {
        Pose2d currentPose = currentPoseSupplier.get();
        this.errorAngle = targetPose.getRotation().minus(currentPose.getRotation()).getMeasure();

        System.out.println(targetPose.minus(currentPose));

        drivetrain.rotateToPose(currentPose, targetPose);

        errorPub.set(errorAngle.in(Degrees));
        currentPub.set(currentPose.getRotation().getDegrees());

        // Pose2d currentPose = currentPoseSupplier.get();
        // Translation2d vectorToTarget = null;

        // if (DriverStation.getAlliance().isPresent() &&
        // DriverStation.getAlliance().get() == Alliance.Blue) {
        // vectorToTarget = GameConstants.blueHubLocation
        // .minus(currentPose.getTranslation());
        // } else if (DriverStation.getAlliance().isPresent() &&
        // DriverStation.getAlliance().get() == Alliance.Red) {
        // vectorToTarget = GameConstants.redHubLocation
        // .minus(currentPose.getTranslation());
        // }

        // Rotation2d targetAngle = vectorToTarget.getAngle();// .plus(new
        // Rotation2d(Radians.of(Math.PI+(Math.PI
        // // /2))));
        // drivetrain.rotateToPose(currentPose, new Pose2d(currentPose.getX(),
        // currentPose.getY(), targetAngle));
    }

    /**
     * Finishes the command when the robot's heading matches the target angle
     * within the tolerance.
     *
     * @return {@code true} if the angular error is within tolerance
     */
    @Override
    public boolean isFinished() {
        return errorAngle.in(Degrees) < kToleranceDegrees;
    }
}
