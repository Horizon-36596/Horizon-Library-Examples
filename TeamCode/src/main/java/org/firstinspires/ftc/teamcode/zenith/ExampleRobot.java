// Copyright (c) 2026 Horizon (FTC 36596). MIT licence: see LICENSE at the repository root.

package org.firstinspires.ftc.teamcode.zenith;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.Path;
import com.seattlesolvers.solverslib.command.Subsystem;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.geometry.Rotation2d;

import org.firstinspires.ftc.teamcode.hardware.ExampleSubsystems;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.HopperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.horizon36596.zenith.solverslib.AutoFromFile;
import org.horizon36596.zenith.solverslib.ZenithRobot;

/**
 * The example robot as the Zenith runtime sees it. zenith.json names this class as
 * {@code deploy.robotClass}, and every OpMode zenith deploy writes builds one.
 */
public class ExampleRobot implements ZenithRobot {

    /** PLACEHOLDER: the same as robot.json's defaultPathSpeedFraction. */
    static final double DEFAULT_SPEED_FRACTION = 0.85;

    private DriveSubsystem drive;
    private IntakeSubsystem intake;
    private LauncherSubsystem launcher;
    private HopperSubsystem hopper;

    @Override
    public void init(AutoFromFile opMode) {
        ExampleSubsystems robot = ExampleSubsystems.create(opMode.hardwareMap);
        drive = robot.drive;
        intake = robot.intake;
        launcher = robot.launcher;
        hopper = robot.hopper;
    }

    @Override
    public void registerCommands(AutoFromFile opMode) {
        ExampleCommands.register(drive, intake, launcher, hopper);
    }

    @Override
    public Follower follower() {
        return drive.follower();
    }

    @Override
    public Subsystem drive() {
        return drive;
    }

    @Override
    public String runningAlliance() {
        return AllianceSelection.current();
    }

    /** BIOBUZZ's field is symmetric about its centre point, so the other alliance's pose is a half turn away. */
    @Override
    public Pose2d mirror(Pose2d pose) {
        return new Pose2d(-pose.getX(), -pose.getY(), new Rotation2d(pose.getHeading() + Math.PI));
    }

    @Override
    public double defaultSpeedFraction() {
        return DEFAULT_SPEED_FRACTION;
    }

    @Override
    public Path atSpeed(Path path, double speedFraction) {
        return drive.atSpeed(path, speedFraction);
    }
}
