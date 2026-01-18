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
		Rotation2d cameraFovUp,
		Rotation2d cameraFovDown
	) {
		this.maxCameraXDistanceFromTag = maxCameraXDistanceFromTag;
		this.mustInRangeX = mustInRangeX;
		this.minCameraXDistanceFromTag = minCameraXDistanceFromTag;

		this.cameraRelativeTag1Height = tagHeight1 - cameraHeight + tagHeightTolerance;
		this.cameraRelativeTag2Height = tagHeight2 - cameraHeight + tagHeightTolerance;

		this.cameraFovUp = cameraFovUp;
		this.cameraFovDown = cameraFovDown;

		this.xRangeFor2Tags = this.maxCameraXDistanceFromTag - minCameraXDistanceFromTag;
	}

	public Rotation2d calculateOptimalPitchFor1Tag(double cameraRelativeTagHeight, double xRangeStart, double xRangeForTag) {
		return calculateOptimalPitch(cameraRelativeTagHeight, calculateRangeEndPoint(xRangeStart, xRangeForTag)).plus(cameraFovDown);
	}

	public Optional<Rotation2d> calculateOptimalPitchFor2Tags() {
		return calculateOptimalSharedPitch(calculateRangeEndPoint(minCameraXDistanceFromTag, xRangeFor2Tags)).isPresent()
			? Optional.of(calculateOptimalSharedPitch(calculateRangeEndPoint(minCameraXDistanceFromTag, xRangeFor2Tags)).get())
			: Optional.empty();
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
			) <= cameraFovUp.plus(cameraFovDown).getDegrees()
		) {
			return Optional.of(
				Rotation2d.fromDegrees(
					(calculateOptimalPitch(cameraRelativeTag1Height, cameraXDistanceFromTag)
						.plus(calculateOptimalPitch(cameraRelativeTag2Height, cameraXDistanceFromTag))
						.plus(cameraFovDown)).getDegrees() / 2
				)
			);
		}
		return Optional.empty();
	}

}
