package frc.utils.brakestate;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.List;

public class BrakeStateManager {

	private enum BrakeMode {
		UNKNOWN,
		BRAKE,
		COAST
	}

	private static final ArrayList<Runnable> brakeRunnables = new ArrayList<>();
	private static final ArrayList<Runnable> coastRunnables = new ArrayList<>();
	private static BrakeMode currentMode = BrakeMode.UNKNOWN;

	public static void add(Runnable brake, Runnable coast) {
		brakeRunnables.add(brake);
		coastRunnables.add(coast);
	}

	private static void setBrakeMode(BrakeMode wantedMode, List<Runnable> setModes) {
		if (currentMode == wantedMode) {
			return;
		}
		currentMode = wantedMode;
		for (Runnable setMode : setModes) {
			setMode.run();
		}
	}

	public static void updateBrakeStateManager(NeutralMode brakeMode) {
		switch (brakeMode) {
			case Brake -> setBrakeMode(BrakeMode.BRAKE, brakeRunnables);
			case Coast -> setBrakeMode(BrakeMode.COAST, coastRunnables);
		}
		log();
	}

	public static void log() {
		Logger.recordOutput("BrakeStateMangerState", currentMode);
	}

}
