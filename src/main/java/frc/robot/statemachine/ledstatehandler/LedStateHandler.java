package frc.robot.statemachine.ledstatehandler;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.led.CANdleWrapper;
import org.littletonrobotics.junction.Logger;

public class LedStateHandler {

	private final String logPath;
	private LedState currentState;
	private CANdleWrapper led;

	public LedStateHandler(String logPath) {
		this.currentState = LedState.NEUTRAL;
		this.logPath = logPath + "/LedStateHandler";
	}

	public LedState getCurrentState() {
		return currentState;
	}

	public Command setState(LedState ledStateHandler) {
		Command command = switch (ledStateHandler) {
			case NEUTRAL, PRE_SCORE, PASS, SCORE, PRE_PASS, STAY_IN_PLACE -> setRBG();
		};
		return new ParallelCommandGroup(
			new InstantCommand(() -> Logger.recordOutput(logPath + "/CurrentState", ledStateHandler.name())),
			new InstantCommand(() -> currentState = ledStateHandler),
			command
		);
	}

	public Command setRBG() {
		return led.asSubsystemCommand(new RunCommand(() -> led.setColor(currentState.getColor())), logPath + "/currentColor");
	}

}

