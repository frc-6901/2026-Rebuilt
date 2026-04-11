package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants.GameConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AlignToTrench extends DriveToTarget {
    public AlignToTrench(
            CommandSwerveDrivetrain drivetrain,
            Supplier<Pose2d> currentPoseSupplier,
            Supplier<SwerveRequest.FieldCentric> driverInputSupplier) {
        super(
                drivetrain,
                currentPoseSupplier, () -> getTrenchAlignedPose(currentPoseSupplier.get()),
                driverInputSupplier, Axis.Y, Axis.THETA);
    }

    private static Pose2d getTrenchAlignedPose(Pose2d currentPose) {
        Distance currentY = currentPose.getMeasureY();
        Distance desiredY;

        double distanceToBlueLeft = currentY.minus(GameConstants.BlueLeftTrenchY).abs(Meters);
        double distanceToBlueRight = currentY.minus(GameConstants.BlueRightTrenchY).abs(Meters);

        if (distanceToBlueLeft < distanceToBlueRight) {
            desiredY = GameConstants.BlueLeftTrenchY;
        } else {
            desiredY = GameConstants.BlueRightTrenchY;
        }

        Rotation2d rotation = currentPose.getRotation();
        double theta = rotation.getDegrees();

        while (theta < 0) {
            theta += 360;
        }
        while (theta >= 360) {
            theta -= 360;
        }

        theta = (theta + 360) % 360;

        if (theta > 180) {
            theta -= 360;
        }

        // theta should be between -180 and 180 at this point

        boolean facingPositiveX = -90 < theta && theta < 90;

        Rotation2d desiredRotation = facingPositiveX ? Rotation2d.fromDegrees(0) : Rotation2d.fromDegrees(180);

        return new Pose2d(currentPose.getMeasureX(), desiredY, desiredRotation);
    }
}