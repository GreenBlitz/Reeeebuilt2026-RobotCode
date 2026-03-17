package frc.constants.field;

public enum AllianceSide {

	DEPOT,
	OUTPOST;

	public static AllianceSide getOppositeSide(AllianceSide allianceSide){
		return switch (allianceSide){
			case DEPOT -> OUTPOST;
			case OUTPOST -> DEPOT;
		};
	}

}
