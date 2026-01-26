package frc.robot.subsystems;

import frc.robot.hardware.empties.EmptyAngleSignal;
import frc.robot.hardware.empties.EmptyControllableMotor;
import frc.robot.hardware.empties.EmptyDoubleSignal;
import frc.robot.hardware.empties.EmptyRequest;
import frc.robot.subsystems.roller.Roller;

public class EmptyBuilder {

	public static Roller buildEmptyRoller(String logPath) {
		return new Roller(
			logPath + "/EmptyRoller",
			new EmptyControllableMotor(),
			new EmptyDoubleSignal(),
			new EmptyDoubleSignal(),
			new EmptyAngleSignal(),
			new EmptyRequest<>()
		);
	}

}
