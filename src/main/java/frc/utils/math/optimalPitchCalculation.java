package frc.utils.math;

import edu.wpi.first.math.geometry.Rotation2d;

public class optimalPitchCalculation {

    private static final double TAG_HEIGHT = 2;
    private static final double CAMERA_HEIGHT = 1;
    private static final double HEIGHT = TAG_HEIGHT - CAMERA_HEIGHT;
    private static final double LIMIT = 4;
    private static final double IMPORTANT = 1;
    private static final double FOV_UP = 10;
    private static final double FOV_DOWN = 11;
    private static final double max = HEIGHT/Math.tan(Rotation2d.fromDegrees(pitch(LIMIT) + FOV_UP + FOV_DOWN).getRadians());
    private static final double range = LIMIT - max;

    public static double pitch(double x){
        return Math.atan2(HEIGHT, x);
    }

    public static double calculateRangeStartingPoint(){
        if(IMPORTANT < max) return IMPORTANT + range;
        return LIMIT;
    }

    public static void main (String[]args){
//        System.out.println(Math.tan(Rotation2d.fromDegrees(pitch(LIMIT) + FOV_UP + FOV_DOWN).getRadians()));
//        System.out.println(pitch(LIMIT));
//        System.out.println(max);
        System.out.println(calculateRangeStartingPoint());
    }
}