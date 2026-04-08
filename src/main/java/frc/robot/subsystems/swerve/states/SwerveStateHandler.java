package frc.robot.subsystems.swerve.states;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.constants.MathConstants;
import frc.constants.field.Field;
import frc.robot.statemachine.StateMachineConstants;
import frc.robot.subsystems.constants.turret.TurretConstants;
import frc.robot.statemachine.ShootingCalculations;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.module.ModuleUtil;
import frc.robot.subsystems.swerve.states.aimassist.AimAssist;
import frc.robot.subsystems.swerve.states.aimassist.AimAssistMath;
import frc.utils.alerts.Alert;
import frc.utils.math.FieldMath;

import java.util.Optional;
import java.util.function.Supplier;

public class SwerveStateHandler {

	private final Swerve swerve;
	private final SwerveConstants swerveConstants;
	private Optional<Supplier<Pose2d>> robotPoseSupplier;
	private Optional<Supplier<Boolean>> isTurretMoveLegalSupplier;
	private Optional<Supplier<Rotation2d>> turretAngleSupplier;
	private boolean isAimAssistOn;
	public Rotation2d towerAimAssistRotationTarget;

	public SwerveStateHandler(Swerve swerve) {
		this.swerve = swerve;
		this.swerveConstants = swerve.getConstants();
		this.robotPoseSupplier = Optional.empty();
		this.isTurretMoveLegalSupplier = Optional.empty();
		this.isAimAssistOn = true;
	}

	public void setRobotPoseSupplier(Supplier<Pose2d> robotPoseSupplier) {
		this.robotPoseSupplier = Optional.of(robotPoseSupplier);
	}

	public void setIsTurretMoveLegalSupplier(Supplier<Boolean> isTurretMoveLegalSupplier) {
		this.isTurretMoveLegalSupplier = Optional.of(isTurretMoveLegalSupplier);
	}

	public void setTurretAngleSupplier(Supplier<Rotation2d> turretAngleSupplier) {
		this.turretAngleSupplier = Optional.of(turretAngleSupplier);
	}

	public void enableAimAssist(boolean enable) {
		isAimAssistOn = enable;
	}

	public ChassisSpeeds applyAimAssistOnChassisSpeeds(ChassisSpeeds speeds, SwerveState swerveState) {
		if (swerveState.getAimAssist() == AimAssist.NONE || !isAimAssistOn) {
			return speeds;
		}
		if (robotPoseSupplier.isEmpty()) {
			reportMissingSupplier("robot pose");
			return speeds;
		}
		if (swerveState.getAimAssist() == AimAssist.TOWER_INTAKE){
			return handleTowerAimAssist(speeds,robotPoseSupplier.get().get(),swerveState);
		}
		if (swerveState.getAimAssist() == AimAssist.LOOK_AT_TARGET) {
			if (isTurretMoveLegalSupplier.isEmpty()) {
				reportMissingSupplier("is turret move legal");
				return speeds;
			}
			if (turretAngleSupplier.isEmpty()) {
				reportMissingSupplier("turret angle");
				return speeds;
			}
			if (isTurretMoveLegalSupplier.get().get() == false) {
				return handleLookAtTargetAimAssist(speeds);
			}
		}
		return speeds;
	}

	private ChassisSpeeds handleLookAtTargetAimAssist(ChassisSpeeds speeds) {
		Pose2d robotPose = robotPoseSupplier.get().get();
		Translation2d target = ShootingCalculations.getShootingParams().targetLandingPosition();
		Rotation2d turretAngle = turretAngleSupplier.get().get();

		double dY = target.getY() - robotPose.getY();
		double dX = target.getX() - robotPose.getX();

		Rotation2d fieldRelativeTurretAngle = turretAngle.plus(robotPose.getRotation());
		Rotation2d targetHeading;
		double joystickRotationalSpeed = 0;

		if (turretAngle.getRotations() < TurretConstants.RANGE_MIDDLE.getRotations()) {
			if (speeds.omegaRadiansPerSecond < 0) {
				joystickRotationalSpeed = speeds.omegaRadiansPerSecond;
			}
			targetHeading = Rotation2d.fromDegrees(
				Rotation2d.fromRadians(Math.atan2(dY, dX)).getDegrees() - StateMachineConstants.DEGREES_OF_OVERSHOOT_FOR_AIM_AT_HUB_ASSIST
			);
		} else {
			if (speeds.omegaRadiansPerSecond > 0) {
				joystickRotationalSpeed = speeds.omegaRadiansPerSecond;
			}
			targetHeading = Rotation2d.fromDegrees(
				Rotation2d.fromRadians(Math.atan2(dY, dX)).getDegrees() + StateMachineConstants.DEGREES_OF_OVERSHOOT_FOR_AIM_AT_HUB_ASSIST
			);
		}

		ChassisSpeeds finalSpeeds = AimAssistMath.getRotationAssistedSpeeds(speeds, fieldRelativeTurretAngle, targetHeading, swerveConstants);
		finalSpeeds.omegaRadiansPerSecond += joystickRotationalSpeed;
		return finalSpeeds;
	}

	private ChassisSpeeds handleTowerAimAssist(ChassisSpeeds speeds,Pose2d robotPose,SwerveState swerveState){
		boolean shouldMirror = robotPose.getX() > Field.LENGTH_METERS / 2;

		Translation2d offset = new Translation2d(Field.TOWER_MIDDLE.getX() / 2, 0);

		Translation2d targetTranslation = Field.TOWER_MIDDLE.minus(offset);

		targetTranslation = FieldMath.mirror(targetTranslation, shouldMirror, shouldMirror);

		return AimAssistMath.getRotationAssistedSpeeds(AimAssistMath.getObjectAssistedSpeeds(speeds,robotPose,Rotation2d.kCW_90deg,targetTranslation,swerveConstants,swerveState),robotPose.getRotation(),towerAimAssistRotationTarget,swerveConstants);
	}

	public Translation2d getRotationAxis(RotateAxis rotationAxisState) {
		return switch (rotationAxisState) {
			case MIDDLE_OF_CHASSIS -> new Translation2d();
			case FRONT_LEFT_MODULE -> swerve.getModules().getModule(ModuleUtil.ModulePosition.FRONT_LEFT).getPositionFromCenterMeters();
			case FRONT_RIGHT_MODULE -> swerve.getModules().getModule(ModuleUtil.ModulePosition.FRONT_RIGHT).getPositionFromCenterMeters();
			case BACK_LEFT_MODULE -> swerve.getModules().getModule(ModuleUtil.ModulePosition.BACK_LEFT).getPositionFromCenterMeters();
			case BACK_RIGHT_MODULE -> swerve.getModules().getModule(ModuleUtil.ModulePosition.BACK_RIGHT).getPositionFromCenterMeters();
		};
	}

	public RotateAxis getFarRotateAxis(boolean isLeft) {
		Rotation2d currentAllianceRelativeHeading = swerve.getAllianceRelativeHeading();
		// -45 <= x <= 45
		if (Math.abs(currentAllianceRelativeHeading.getDegrees()) <= MathConstants.EIGHTH_CIRCLE.getDegrees()) {
			return isLeft ? RotateAxis.FRONT_LEFT_MODULE : RotateAxis.FRONT_RIGHT_MODULE;
		}
		// -180 <= x <= -135 || 135 <= x <= 180
		if (Math.abs(currentAllianceRelativeHeading.getDegrees()) >= MathConstants.EIGHTH_CIRCLE.getDegrees() * 3) {
			return isLeft ? RotateAxis.BACK_RIGHT_MODULE : RotateAxis.BACK_LEFT_MODULE;
		}
		// 45 <= x <= 135
		if (currentAllianceRelativeHeading.getDegrees() > 0) {
			return isLeft ? RotateAxis.FRONT_RIGHT_MODULE : RotateAxis.BACK_RIGHT_MODULE;
		}
		// -45 >= x >= -135
		return isLeft ? RotateAxis.BACK_LEFT_MODULE : RotateAxis.FRONT_LEFT_MODULE;
	}

	private void reportMissingSupplier(String supplierName) {
		new Alert(Alert.AlertType.WARNING, swerve.getLogPath() + "/AimAssist/missing" + supplierName + " supplier").report();
	}

	public RotateAxis getFarRightRotateAxis() {
		return getFarRotateAxis(false);
	}

	public RotateAxis getFarLeftRotateAxis() {
		return getFarRotateAxis(true);
	}

}
