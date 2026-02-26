package frc.robot.statemachine.ledstatehandler;

import com.ctre.phoenix6.signals.RGBWColor;
import edu.wpi.first.wpilibj.util.Color;

public enum LedState {

	STAY_IN_PLACE(new RGBWColor(Color.kWhite)),
	NEUTRAL(new RGBWColor(Color.kLimeGreen)),
	PRE_SCORE(new RGBWColor(Color.kSkyBlue)),
	SCORE(new RGBWColor(Color.kRoyalBlue)),
	PRE_PASS(new RGBWColor(Color.kLightPink)),
	PASS(new RGBWColor(Color.kIndianRed));

	public final RGBWColor color;

	LedState(RGBWColor color) {
		this.color = color;
	}

	public RGBWColor getColor() {
		return color;
	}

}
