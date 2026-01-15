// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc;

import edu.wpi.first.math.geometry.Translation2d;
import frc.utils.math.StatisticsMath;
import frc.utils.math.ToleranceMath;

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
//		RobotBase.startRobot(RobotManager::new);
		Translation2d[] a = new Translation2d[] {
			new Translation2d(5, 5),
			new Translation2d(5, 7),
			new Translation2d(5, 4.6),
			new Translation2d(5, 5),};
		Translation2d maj = StatisticsMath.getMajority(a, 0);
		System.out.println(maj);

		boolean is[] = new boolean[4];
		for (int i = 0; i < a.length; i++) {
			is[i] = !ToleranceMath.isNear(maj, a[i], 0.5);
		}
		System.out.println(is[0]);
		System.out.println(is[1]);
		System.out.println(is[2]);
		System.out.println(is[3]);
	}

}
