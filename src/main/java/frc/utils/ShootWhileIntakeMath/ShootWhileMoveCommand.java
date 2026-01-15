//package frc.utils.ShootWhileIntakeMath;
//
//import edu.wpi.first.math.geometry.Rotation2d;
//import edu.wpi.first.math.geometry.Translation2d;
//import edu.wpi.first.math.interpolation.Interpolator;
//import edu.wpi.first.math.interpolation.InverseInterpolator;
//import edu.wpi.first.wpilibj2.command.Command;
//import frc.constants.field.Field;
//import frc.robot.Robot;
//
//import frc.utils.InterpolationMap;
//
//import java.util.Map;
//
//public class ShootWhileMoveCommand extends Command {
//
//    private final Robot robot;
//
//    private static final InterpolationMap<Double, Double> TIME_INTERPOLATION_MAP = new InterpolationMap<Double, Double>(
//            InverseInterpolator.forDouble(),
//            Interpolator.forDouble(),
//            Map.of(
//                    0.8,
//                    5.0
//            )
//    );
//
//    public ShootWhileMoveCommand(Robot robot){
//        this.robot = robot;
//        addRequirements(robot.getFlyWheel(), robot.getTurret(), robot.getHood());
//    }
//
//    @Override
//    public void execute(){
//        Translation2d target = Field.getHubMiddle();
//
//        Translation2d robotToGoal = target.minus(robot.getPoseEstimator().getEstimatedPose().getTranslation());
//        double dist = robotToGoal.getDistance(new Translation2d());
//
//        double fixedShotTime = TIME_INTERPOLATION_MAP.get(dist);
//
//        double virtualGoalX = target.getX()-fixedShotTime*(robot.getSwerve().getAllianceRelativeVelocity().vxMetersPerSecond+robot.getSwerve().getAccelerationFromIMUMetersPerSecondSquared().getX()*1);
//        double virtualGoalY = target.getY()-fixedShotTime*(robot.getSwerve().getAllianceRelativeVelocity().vyMetersPerSecond+robot.getSwerve().getAccelerationFromIMUMetersPerSecondSquared().getY()*1);
//
//        Translation2d movingGoalLocation = new Translation2d(virtualGoalX,virtualGoalY);
//
//        Translation2d toMovingGoal = movingGoalLocation.minus(robot.getPoseEstimator().getEstimatedPose().getTranslation());
//
//        double newDist = toMovingGoal.getDistance(new Translation2d());
//
//        flyWheel.run(m_rpmTable.getOutput(newDist));
//        hood.run(m_hoodTable.getOutput(newDist));
//    }
//
//
//    @Override
//    public void end(boolean interrupted) {
//        turret.trackTarget(false);
//        turret.disable();
//        turret.stop();
//        flyWheel.stop();
//        hood.stop();
//    }
//
//    private Pose2d calcPoseFromVision(double dL, double tR, double tT, double tL, Translation2d goal){
//        double tG = tR+tT+tL;
//        double rX = goal.getX()-dL*Math.cos(tG);
//        double rY = goal.getY()-dL*Math.sin(tG);
//
//        return new Pose2d(rX,rY, new Rotation2d(-tR));
//    }
//
//}