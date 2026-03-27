package frc.robot.statemachine;

import frc.robot.subsystems.swerve.states.DriveSpeed;
import frc.robot.subsystems.swerve.states.SwerveState;
import frc.robot.subsystems.swerve.states.aimassist.AimAssist;
import frc.robot.subsystems.swerve.states.AccelerationLimit;


public enum RobotState {

	STAY_IN_PLACE,
	NEUTRAL(SwerveState.DEFAULT_DRIVE.withAccelerationLimit(AccelerationLimit.SHOOTING)),
	OUTTAKE,
	PRE_SCORE(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.LOOK_AT_TARGET).withDriveSpeed(DriveSpeed.SHOOT)),
	SCORE(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.LOOK_AT_TARGET).withDriveSpeed(DriveSpeed.SHOOT).withAccelerationLimit(AccelerationLimit.SHOOTING)),
	PRE_PASS(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.LOOK_AT_TARGET).withDriveSpeed(DriveSpeed.SHOOT)),
	PASS(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.LOOK_AT_TARGET).withDriveSpeed(DriveSpeed.SHOOT)),
	RESET_SUBSYSTEMS,
	CALIBRATION_PRE_SCORE,
	CALIBRATION_SCORE;

	private final SwerveState swerveState;
	private boolean swerveStateActive;

	RobotState() {
		this(SwerveState.DEFAULT_DRIVE);
	}

	RobotState(SwerveState state) {
		this.swerveState = state;
		this.swerveStateActive = true;
	}

	public RobotState activateSwerveAssist(boolean active) {
		swerveStateActive = active;
		return this;
	}

	public SwerveState getSwerveState() {
		return swerveStateActive ? swerveState : SwerveState.DEFAULT_DRIVE;
	}

}
