package frc.utils.math;

import edu.wpi.first.math.geometry.Rotation2d;

import java.util.Optional;

public class OptimalCameraPitchCalculation {

	private final double maxCameraXDistanceFromTag;
	private final double minCameraXDistanceFromTag;
	private final double mustInRangeX;

	private final double cameraRelativeTag1Height;
	private final double cameraRelativeTag2Height;

	private final Rotation2d cameraFovUp;
	private final Rotation2d cameraFovDown;
	private final double xRangeFor2Tags;

	public OptimalCameraPitchCalculation(
		double maxCameraXDistanceFromTag,
		double mustInRangeX,
		double minCameraXDistanceFromTag,
		double tagHeight1,
		double tagHeight2,
		double tagHeightTolerance,
		double cameraHeight,
		Rotation2d cameraFovUpInDegrees,
		Rotation2d cameraFovDownInDegrees
	) {
		this.maxCameraXDistanceFromTag = maxCameraXDistanceFromTag;
		this.mustInRangeX = mustInRangeX;
		this.minCameraXDistanceFromTag = minCameraXDistanceFromTag;

		this.cameraRelativeTag1Height = tagHeight1 - cameraHeight + tagHeightTolerance;
		this.cameraRelativeTag2Height = tagHeight2 - cameraHeight + tagHeightTolerance;

		this.cameraFovUp = cameraFovUpInDegrees;
		this.cameraFovDown = cameraFovDownInDegrees;

		this.xRangeFor2Tags = this.maxCameraXDistanceFromTag - minCameraXDistanceFromTag;
	}

	public Rotation2d calculateOptimalPitchFor1Tag(double cameraRelativeTagHeight, double xRangeStart, double xRangeForTag) {
		return Rotation2d.fromDegrees(
			calculateOptimalPitch(cameraRelativeTagHeight, calculateRangeEndPoint(xRangeStart, xRangeForTag)).getDegrees()
				+ cameraFovDown.getDegrees()
		);
	}

	public Optional<Rotation2d> calculateOptimalPitchFor2Tags() {
		return calculateOptimalSharedPitch(calculateRangeEndPoint(minCameraXDistanceFromTag, xRangeFor2Tags));
	}

	private Rotation2d calculateOptimalPitch(double cameraRelativeTagHeight, double cameraXDistanceFromTag) {
		return Rotation2d.fromRadians(Math.atan2(cameraRelativeTagHeight, cameraXDistanceFromTag));
	}

	private double calculateRangeEndPoint(double xRangeStart, double xRangeForTag) {
		if (mustInRangeX < xRangeStart)
			return mustInRangeX + xRangeForTag;
		return maxCameraXDistanceFromTag;
	}

	private Optional<Rotation2d> calculateOptimalSharedPitch(double cameraXDistanceFromTag) {
		if (
			Math.abs(
				calculateOptimalPitch(cameraRelativeTag1Height, cameraXDistanceFromTag).getDegrees()
					- calculateOptimalPitch(cameraRelativeTag2Height, cameraXDistanceFromTag).getDegrees()
			) <= cameraFovUp.getDegrees() + cameraFovDown.getDegrees()
		) {
			return Optional.of(
				Rotation2d.fromDegrees(
					calculateOptimalPitch(cameraRelativeTag1Height, cameraXDistanceFromTag).getDegrees()
						+ calculateOptimalPitch(cameraRelativeTag2Height, cameraXDistanceFromTag).getDegrees() / 2
				)
			);
		}
		return Optional.empty();
	}

}
