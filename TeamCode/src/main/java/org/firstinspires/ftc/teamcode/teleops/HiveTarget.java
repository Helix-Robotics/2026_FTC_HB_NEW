package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.List;

final class HiveTarget {
    // Crossbotics TagGroup. The four tag positions in inches from the row center.
    private static final double[] TAG_OFFSETS = {-6.5, -2.75, 2.75, 6.5};

    enum AimPoint {
        LEFT(-2.75), CENTER(0), RIGHT(2.75);

        final double inchesFromCenter;

        AimPoint(double inchesFromCenter) {
            this.inchesFromCenter = inchesFromCenter;
        }
    }

    private final double distanceDifferenceInches;
    private final double[] angles;

    private HiveTarget(double distanceDifferenceInches, double[] angles) {
        this.distanceDifferenceInches = distanceDifferenceInches;
        this.angles = angles;
    }

    static HiveTarget find(List<FiducialResult> tags, boolean blue) {
        if (tags == null) {
            return null;
        }
        int firstTag = blue ? 38 : 30;
        HiveTarget firstCell = findCell(tags, firstTag);
        HiveTarget secondCell = findCell(tags, firstTag + TAG_OFFSETS.length);
        if (firstCell != null && secondCell != null) {
            return null;
        }
        return firstCell != null ? firstCell : secondCell;
    }

    private static HiveTarget findCell(List<FiducialResult> tags, int firstTag) {
        Position[] positions = new Position[TAG_OFFSETS.length];
        Position left = null;
        Position right = null;
        int leftIndex = TAG_OFFSETS.length;
        int rightIndex = -1;

        for (FiducialResult tag : tags) {
            int index = tag.getFiducialId() - firstTag;
            if (index < 0 || index >= TAG_OFFSETS.length) {
                continue;
            }
            Pose3D pose = tag.getTargetPoseCameraSpace();
            if (pose == null) {
                continue;
            }
            Position position = pose.getPosition();
            if (!Double.isFinite(position.x) || !Double.isFinite(position.y)
                    || !Double.isFinite(position.z) || position.z <= 0) {
                continue;
            }
            positions[index] = position;
            if (index < leftIndex) {
                left = position;
                leftIndex = index;
            }
            if (index > rightIndex) {
                right = position;
                rightIndex = index;
            }
        }

        if (leftIndex >= rightIndex || right.x <= left.x) {
            return null;
        }

        // Use the widest visible pair to estimate the row center and any hidden tags.
        double leftOffset = TAG_OFFSETS[leftIndex];
        double tagSpacing = TAG_OFFSETS[rightIndex] - leftOffset;
        double xPerInch = (right.x - left.x) / tagSpacing;
        double yPerInch = (right.y - left.y) / tagSpacing;
        double zPerInch = (right.z - left.z) / tagSpacing;
        double centerX = left.x - leftOffset * xPerInch;
        double centerY = left.y - leftOffset * yPerInch;
        double centerZ = left.z - leftOffset * zPerInch;
        double halfWidth = TAG_OFFSETS[TAG_OFFSETS.length - 1];
        double leftDistance = Math.hypot(Math.hypot(centerX - halfWidth * xPerInch,
                centerY - halfWidth * yPerInch), centerZ - halfWidth * zPerInch);
        double rightDistance = Math.hypot(Math.hypot(centerX + halfWidth * xPerInch,
                centerY + halfWidth * yPerInch), centerZ + halfWidth * zPerInch);
        double distanceDifference = left.unit.toInches(rightDistance - leftDistance);

        AimPoint[] aimPoints = AimPoint.values();
        double[] angles = new double[aimPoints.length];
        for (AimPoint aim : aimPoints) {
            Position innerTag = aim == AimPoint.LEFT ? positions[1]
                    : aim == AimPoint.RIGHT ? positions[2] : null;
            double aimX = innerTag != null ? innerTag.x : centerX + aim.inchesFromCenter * xPerInch;
            double aimZ = innerTag != null ? innerTag.z : centerZ + aim.inchesFromCenter * zPerInch;
            if (aimZ <= 0) {
                return null;
            }
            angles[aim.ordinal()] = Math.toDegrees(Math.atan2(aimX, aimZ));
        }
        return new HiveTarget(distanceDifference, angles);
    }

    double angleTo(AimPoint aimPoint) {
        return angles[aimPoint.ordinal()];
    }

    AimPoint chooseAimPoint(AimPoint previous, double cornerDifference) {
        double difference = distanceDifferenceInches;
        // A smaller exit threshold keeps noisy readings from bouncing between a corner and center.
        double returnToCenter = cornerDifference * 2 / 3;
        if (difference > cornerDifference || (previous == AimPoint.LEFT && difference > returnToCenter)) {
            return AimPoint.LEFT;
        }
        if (difference < -cornerDifference || (previous == AimPoint.RIGHT && difference < -returnToCenter)) {
            return AimPoint.RIGHT;
        }
        return AimPoint.CENTER;
    }
}
