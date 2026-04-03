package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class DriveToTarget extends Command {
    private final CommandSwerveDrivetrain drivetrain;

    private final Supplier<Pose2d> currentPoseSupplier;
    private final Supplier<Pose2d> targetPoseSupplier;
    private Pose2d targetPose;

    private final boolean[] controllersToUse;
    private final Supplier<SwerveRequest.FieldCentric> driverInput;

    public final Angle ThetaErrorTolerance = Degrees.of(10);
    public final Distance DisplacementErrorTolerance = Meters.of(0.5);

    private Angle thetaError;
    private Distance displacementError;

    private boolean isCompleted;

    private final DoublePublisher thetaErrorPub = NetworkTableInstance.getDefault()
            .getTable("DriveToTarget")
            .getDoubleTopic("Error (theta)")
            .publish();

    private final DoublePublisher displacementErrorPub = NetworkTableInstance.getDefault()
            .getTable("DriveToTarget")
            .getDoubleTopic("Error (displacement)")
            .publish();

    public DriveToTarget(
            CommandSwerveDrivetrain drivetrain,
            Supplier<Pose2d> currentPoseSupplier,
            Supplier<Pose2d> targetPoseSupplier,
            boolean[] controllersToUse,
            Supplier<SwerveRequest.FieldCentric> driverInput) {
        this.drivetrain = drivetrain;
        this.currentPoseSupplier = currentPoseSupplier;
        this.targetPoseSupplier = targetPoseSupplier;
        this.controllersToUse = controllersToUse;
        this.driverInput = driverInput;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        targetPose = targetPoseSupplier.get();
        drivetrain.resetPIDControllers();

        isCompleted = false;
    }

    @Override
    public void execute() {
        Pose2d currentPose = currentPoseSupplier.get();

        thetaError = currentPose.getRotation().minus(targetPose.getRotation()).getMeasure();
        displacementError = Meters.of(currentPose.getTranslation().getDistance(targetPose.getTranslation()));

        isCompleted = thetaError.abs(Degrees) <= ThetaErrorTolerance.in(Degrees)
                && displacementError.abs(Meters) <= DisplacementErrorTolerance.in(Meters);
        if (!isCompleted) {
            drivetrain.driveToPose(currentPose, targetPose, controllersToUse, driverInput.get());
        }

        thetaErrorPub.set(thetaError.in(Degrees));
        displacementErrorPub.set(displacementError.in(Meters));
    }

    @Override
    public boolean isFinished() {
        return isCompleted;
    }
}
