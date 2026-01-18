package frc.utils.math;

import edu.wpi.first.math.geometry.Rotation2d;

import java.util.Optional;

public class OptimalCameraPitchCalculation {

	private final double essentialCameraXDistanceFromTag;
	private final double maxCameraXDistanceFromTag;

	private final double cameraRelativeTag1Height;
	private final double cameraRelativeTag2Height;

	private final Rotation2d cameraFovUp;
	private final Rotation2d cameraFovDown;

	public OptimalCameraPitchCalculation(
		double essentialCameraXDistanceFromTag,
		double maxCameraXDistanceFromTag,
		double tagHeight1,
		double tagHeight2,
		double tagHeightTolerance,
		double cameraHeight,
		Rotation2d cameraFovUp,
		Rotation2d cameraFovDown
	) {
		this.essentialCameraXDistanceFromTag = essentialCameraXDistanceFromTag;
		this.maxCameraXDistanceFromTag = maxCameraXDistanceFromTag;

		this.cameraRelativeTag1Height = tagHeight1 - cameraHeight + tagHeightTolerance;
		this.cameraRelativeTag2Height = tagHeight2 - cameraHeight + tagHeightTolerance;

		this.cameraFovUp = cameraFovUp;
		this.cameraFovDown = cameraFovDown;
	}

	public Rotation2d calculateOptimalPitchFor1Tag(double cameraRelativeTagHeight, double xRangeStart, double xRangeForTag) {
		return calculateOptimalPitch(cameraRelativeTagHeight, xRangeStart, xRangeForTag);
	}

	public Optional<Rotation2d> calculateOptimalPitchFor2Tags(
		double xRangeStart,
		double xRangeForTag,
		double minCameraXDistanceFromTag,
		double xRangeFor2Tags
	) {
		return calculateOptimalSharedPitch(calculateRangeEndPoint(minCameraXDistanceFromTag, xRangeFor2Tags), xRangeStart, xRangeForTag)
			.isPresent()
				? Optional.of(
					calculateOptimalSharedPitch(calculateRangeEndPoint(minCameraXDistanceFromTag, xRangeFor2Tags), xRangeStart, xRangeForTag)
						.get()
				)
				: Optional.empty();
	}

	private Rotation2d calculatePitch(double cameraRelativeTagHeight, double cameraXDistanceFromTag) {
		return Rotation2d.fromRadians(Math.atan2(cameraRelativeTagHeight, cameraXDistanceFromTag));
	}

	private Rotation2d calculateOptimalPitch(double cameraRelativeTagHeight, double xRangeStart, double xRangeForTag) {
		return calculatePitch(cameraRelativeTagHeight, calculateRangeEndPoint(xRangeStart, xRangeForTag)).plus(cameraFovDown);
	}

	private double calculateRangeEndPoint(double xRangeStart, double xRangeForTag) {
		if (essentialCameraXDistanceFromTag < xRangeStart)
			return essentialCameraXDistanceFromTag + xRangeForTag;
		return maxCameraXDistanceFromTag;
	}

	public Optional<Rotation2d> calculateOptimalSharedPitch(double cameraXDistanceFromTag, double xRangeStart, double xRangeForTag) {
		if (
			Math.abs(
				calculateOptimalPitch(cameraRelativeTag1Height, xRangeStart, xRangeForTag).getDegrees()
					- calculateOptimalPitch(cameraRelativeTag2Height, xRangeStart, xRangeForTag).getDegrees()
			) <= cameraFovUp.plus(cameraFovDown).getDegrees()
		) {
			System.out.println(("pitch1 " + calculateOptimalPitch(cameraRelativeTag1Height, xRangeStart, xRangeForTag)));
			System.out.println(("pitch2 " + calculateOptimalPitch(cameraRelativeTag2Height, xRangeStart, xRangeForTag)));
			return Optional.of(
				calculateOptimalPitch(cameraRelativeTag1Height, xRangeStart, xRangeForTag)
					.plus(calculateOptimalPitch(cameraRelativeTag2Height, xRangeStart, xRangeForTag))
					.div(2)
			);
		}
		return Optional.empty();
	}

}
