package frc.robot.statemachine.shooterstatehandler;

import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.math.geometry.Translation2d;

public record ShootingParams(
	Rotation2d targetFlywheelVelocityRPS,
	Rotation2d targetHoodPosition,
	Rotation2d targetTurretPosition,
	Rotation2d targetTurretVelocityRPS,
	Translation2d predictedTurretPoseWhenBallLands,
	Translation2d targetLandingPosition
) {}
