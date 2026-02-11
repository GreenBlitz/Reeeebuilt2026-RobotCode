// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.RobotBase;
import frc.utils.limelight.LimelightHelpers;
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
    Transform2d camTransformRelativeToRobotCenter = new Transform2d(0,1,(Rotation2d.fromDegrees(0)));
    //Transform2d = new Transform2d(())
    Pose2d limeLightPoseRelativeToField = new Pose2d(2,3.0,Rotation2d.fromDegrees(180));
    Pose2d robotCenterPoseRelativeToField = getRobotPoseFromTurretLimeLight(camTransformRelativeToRobotCenter, limeLightPoseRelativeToField);
    System.out.println(robotCenterPoseRelativeToField.toString());
    }
    public static Pose2d getRobotPoseFromTurretLimeLight(Transform2d camTransformRelativeToRobotCenter ,Pose2d limeLightPoseRelativeToField ) {
        // important remember that the pose the limelight helpers return are shifted by the given pose in the constructor
        // to avoid shiftinge at the constructor use LimelightHelpers.setCameraPose_RobotSpace when pose is set to zero
        // what transform 2d does is set the angle to 0 like a new axis were working on

        return (limeLightPoseRelativeToField.transformBy(camTransformRelativeToRobotCenter.inverse()));

    }
}
