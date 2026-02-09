// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.RobotBase;
import frc.utils.limelight.LimelightHelpers;
import
/**
 * Do NOT add any static variables to this class, or any initialization at all. Unless you know what you are doing, do not modify this file
 * except to change the parameter class to the startRobot call.
 */
public final class Main {

	/**
	 * Main initialization function. Do not perform any initialization here.
	 *
	 * <p>If you change your main robot class, change the parameter type.
	 */
	public static void main(String... args) {
    Pose2d pose = getRobotPoseFromTurretLimeLight(new Transform2d(2.0,2.0,new Rotation2d(1.0,0.0)),"name");
	pose.toString();
    }
    public static Pose2d getRobotPoseFromTurretLimeLight(Transform2d camTransformRelativeToRobotCenter, String cameraName) {
        // important remember that the pose the limelight helpers return are shifted by the given pose in the constructor
        // to avoid shiftinge at the constructor use LimelightHelpers.setCameraPose_RobotSpace when pose is set to zero
        // what transform 2d does is set the angle to 0 like a new axis were working on

        Pose2d limeLightPoseRelativeToField = new Pose2d(2.0,4.0,new Rotation2d(0.0,1.0));
        return (limeLightPoseRelativeToField.transformBy(camTransformRelativeToRobotCenter.inverse()));

    }
}
