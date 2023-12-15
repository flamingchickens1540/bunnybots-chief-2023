package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.limelight.Limelight;
import frc.robot.subsystems.shooter.Shooter;

import java.util.HashMap;
import java.util.List;

public class PathPlannerDriveCommand extends SequentialCommandGroup {
    public PathPlannerDriveCommand(String path, Drive drivetrain, Shooter shooter, Limelight limelight) {
        addRequirements(drivetrain);


        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(path, new PathConstraints(4, 3));

        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("shoot 1", new ShooterCommands.Shoot(shooter, limelight));
        eventMap.put("shoot 5", new SequentialCommandGroup(
                new ShooterCommands.Shoot(shooter, limelight),
                new ShooterCommands.Shoot(shooter, limelight),
                new ShooterCommands.Shoot(shooter, limelight),
                new ShooterCommands.Shoot(shooter, limelight),
                new ShooterCommands.Shoot(shooter, limelight)
        ));




        SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
                drivetrain::getPose, // Pose2d supplier
                drivetrain::setPose, // Pose2d consumer, used to reset odometry at the beginning of auto
                drivetrain.getKinematics(), // SwerveDriveKinematics
                new PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
                new PIDConstants(0.5, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
                drivetrain::setModuleStates, // Module states consumer used to output to the drive subsystem
                eventMap,
                true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
                drivetrain // The drive subsystem. Used to properly set the requirements of path following commands
        );

        addCommands(
             autoBuilder.followPathGroup(pathGroup)
        );
    }
}
