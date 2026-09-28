// Copyright (c) 2026 Horizon (FTC 36596). MIT licence: see LICENSE at the repository root.

package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.Path;
import com.seattlesolvers.solverslib.command.SubsystemBase;

/**
 * The mecanum drive, as the Pedro Pathing follower drives it. Every Zenith path step requires this
 * subsystem, so two paths never drive at once.
 */
public class DriveSubsystem extends SubsystemBase {

    private final Follower follower;
    private final ForesightConfig config;

    /**
     * @param follower the Pedro follower for this drive
     * @param config   the follower's algorithm settings, used to cap a path's speed
     */
    public DriveSubsystem(Follower follower, ForesightConfig config) {
        this.follower = follower;
        this.config = config;
    }

    public Follower follower() {
        return follower;
    }

    /** The follower updates once per loop, here, because the SolversLib scheduler calls it every loop. */
    @Override
    public void periodic() {
        follower.update();
    }

    /**
     * Caps a path to a fraction of the robot's top speed while it is followed.
     *
     * @param path          the path about to be followed
     * @param speedFraction 0 to 1
     * @return the same path with the cap attached
     */
    public Path atSpeed(Path path, double speedFraction) {
        return path.with(config.setPathSpeed(speedFraction));
    }

    /**
     * Drives by hand, robot-centric, in Pedro's power units: forward, to the robot's left, and
     * counter-clockwise, each -1 to 1. The follower keeps doing it until a path or a hold replaces it.
     */
    public void driveManual(DrivePowers powers) {
        follower.manual(powers);
    }

    /**
     * Where a robot with a camera would correct its pose from a field target. This robot has only
     * odometry, so the follower keeps the pose it has.
     */
    public void relocalize() {
        // Nothing to correct from: the odometry pose is already the follower's pose.
    }
}
