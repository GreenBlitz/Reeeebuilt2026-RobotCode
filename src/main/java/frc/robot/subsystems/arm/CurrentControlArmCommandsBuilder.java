package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;

import java.util.function.Supplier;

public class CurrentControlArmCommandsBuilder extends ArmCommandBuilder {

	private final CurrentControlArm arm;

	public CurrentControlArmCommandsBuilder(CurrentControlArm arm) {
		super(arm);
		this.arm = arm;
	}

	public Command setCurrent(double current) {
		return arm.asSubsystemCommand(new RunCommand(() -> arm.setCurrent(current)), "Set current to: " + current);
	}

	public Command setCurrent(Supplier<Double> current) {
		return arm.asSubsystemCommand(new RunCommand(() -> arm.setCurrent(current.get())), "Set current by supplier");
	}

}
