package frc.constants.field;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.utils.DriverStationUtil;
import frc.utils.math.AngleTransform;
import frc.utils.math.FieldMath;

public class Field {

	public static final Translation2d Tower = new Translation2d(6.5, 4);

	public static final double TOWER_SIDES_DISTANCE_FROM_MIDDLE_METERS = 0.27;

	public static final DriverStation.Alliance RELATIVE_FIELD_CONVENTION_ALLIANCE = DriverStation.Alliance.Blue;

	public static boolean isFieldConventionAlliance() {
		return DriverStationUtil.getAlliance() == RELATIVE_FIELD_CONVENTION_ALLIANCE;
	}

	public static final double LENGTH_METERS = 16.54048;
	public static final double WIDTH_METERS = 8.06958;

	private static final Translation2d HUB_MIDDLE = new Translation2d(4.62, 4.03);

	private static final Translation2d DEPOT_MIDDLE = new Translation2d(0.31, 5.97);
	public static final double DEPOT_Y_AXIS_LENGTH_METERS = 1.0668;
	public static final double DEPOT_X_AXIS_LENGTH_METERS = 0.6858;

	private static final Translation2d OUTPOST_MIDDLE = new Translation2d(0, 0.67);

	private static final Translation2d TOWER_MIDDLE = new Translation2d(1.06, 3.75);

	private static final Translation2d DEPOT_TRENCH_MIDDLE = new Translation2d(4.62, 7.43);
	private static final Translation2d OUTPOST_TRENCH_MIDDLE = new Translation2d(4.62, 0.64);

	private static final Translation2d DEPOT_BUMP_MIDDLE = new Translation2d(4.62, 5.56);
	private static final Translation2d OUTPOST_BUMP_MIDDLE = new Translation2d(4.62, 2.51);


	public static Translation2d getHubMiddle() {
		return getAllianceRelative(HUB_MIDDLE);
	}

	public static Translation2d getDepotMiddle() {
		return getAllianceRelative(DEPOT_MIDDLE);
	}

	public static Translation2d getOutpostMiddle() {
		return getAllianceRelative(OUTPOST_MIDDLE);
	}

	public static Translation2d getTowerMiddle() {
		return getAllianceRelative(TOWER_MIDDLE);
	}

	public static Translation2d getTrenchMiddle(AllianceSide side) {
		return getAllianceRelative(switch (side) {
			case DEPOT -> DEPOT_TRENCH_MIDDLE;
			case OUTPOST -> OUTPOST_TRENCH_MIDDLE;
		});
	}

	public static Translation2d getBumpMiddle(AllianceSide side) {
		return getAllianceRelative(switch (side) {
			case DEPOT -> DEPOT_BUMP_MIDDLE;
			case OUTPOST -> OUTPOST_BUMP_MIDDLE;
		});
	}

	public static Pose2d getAllianceRelative(Pose2d pose2d) {
		return new Pose2d(getAllianceRelative(pose2d.getTranslation()), getAllianceRelative(pose2d.getRotation()));
	}

	public static Translation2d getAllianceRelative(Translation2d translation) {
		return isFieldConventionAlliance() ? translation : FieldMath.mirror(translation, true, true);
	}

	public static Rotation2d getAllianceRelative(Rotation2d rotation) {
		return isFieldConventionAlliance() ? rotation : FieldMath.transformAngle(rotation, AngleTransform.INVERT);
	}

	public static Pose3d getAllianceRelative(Pose3d pose) {
		return new Pose3d(getAllianceRelative(pose.getTranslation()), getAllianceRelative(pose.getRotation()));
	}

	public static Translation3d getAllianceRelative(Translation3d translation) {
		return isFieldConventionAlliance() ? translation : FieldMath.mirror(translation, true, true);
	}

	public static Rotation3d getAllianceRelative(Rotation3d rotation) {
		return isFieldConventionAlliance()
			? rotation
			: FieldMath.transformAngle(rotation, AngleTransform.KEEP, AngleTransform.KEEP, AngleTransform.INVERT);
	}

}
