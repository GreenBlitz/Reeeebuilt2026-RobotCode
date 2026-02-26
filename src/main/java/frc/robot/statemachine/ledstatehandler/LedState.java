package frc.robot.statemachine.ledstatehandler;

import com.ctre.phoenix6.signals.RGBWColor;
import edu.wpi.first.wpilibj.util.Color;

public enum LedState {

	NEUTRAL(new RGBWColor(Color.kRoyalBlue)),
	PRE_SCORE(new RGBWColor(Color.kSkyBlue)),
	SCORE(new RGBWColor(Color.kRoyalBlue)),
	PRE_PASS(new RGBWColor(Color.kLightPink)),
	PASS(new RGBWColor(Color.kIndianRed)),
	STAY_IN_PLACE(new RGBWColor(Color.kLightYellow)),
	RESET_SUBSYSTEMS(new RGBWColor(Color.kDarkSlateBlue)),
	CALIBRATION_PRE_SCORE(new RGBWColor(Color.kBrown)),
	CALIBRATION_SCORE(new RGBWColor(Color.kBlueViolet));

	public final RGBWColor color;

	LedState(RGBWColor color) {
		this.color = color;
	}

	public RGBWColor getColor() {
		return color;
	}

}
