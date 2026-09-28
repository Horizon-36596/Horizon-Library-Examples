// Copyright (c) 2026 Horizon (FTC 36596). MIT licence: see LICENSE at the repository root.

package org.firstinspires.ftc.teamcode.hardware;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Builds the Pedro Pathing v3 follower for the mecanum drive.
 *
 * <p>Every number here is a PLACEHOLDER. Tune your own robot with Pedro's tuners and copy the results
 * in. Keep {@link #MAX_FORWARD_IN_PER_S}, {@link #MAX_STRAFE_IN_PER_S} and the decelerations equal to
 * the {@code kinematics} block of {@code autos/robot.json}, so Zenith's estimates match the robot.
 */
public final class PedroSetup {

    /** PLACEHOLDER: the same as robot.json's maxForwardVelInPerS. */
    public static final double MAX_FORWARD_IN_PER_S = 70;
    /** PLACEHOLDER: the same as robot.json's maxStrafeVelInPerS. */
    public static final double MAX_STRAFE_IN_PER_S = 56;
    /** PLACEHOLDER: the same as robot.json's forwardDecelInPerS2. */
    public static final double FORWARD_DECEL_IN_PER_S2 = 80;
    /** PLACEHOLDER: the same as robot.json's strafeDecelInPerS2. */
    public static final double STRAFE_DECEL_IN_PER_S2 = 55;

    /** PLACEHOLDER: odometry pod offsets from the robot centre, inches. */
    private static final double X_POD_OFFSET_IN = -3.5;
    private static final double Y_POD_OFFSET_IN = -6.0;

    private PedroSetup() {}

    /**
     * The follower's algorithm settings. Kept so a path can be capped to a speed fraction. The
     * controller gains are Pedro's defaults; set the rest from Pedro's tuners.
     */
    public static ForesightConfig foresightConfig() {
        return new ForesightConfig(c -> {
            // PLACEHOLDER braking model: forward, then strafe.
            c.linearBrakeCoefficients.set(Matrix.diag(0.08, 0.1));
            c.quadraticBrakeCoefficients.set(Matrix.diag(0.0015, 0.002));
            c.headingBrakeCoefficients.set(Vector2D.cartesian(0.06, 0.004));
            c.maxAchievableForwardVelocity.set(MAX_FORWARD_IN_PER_S);
            c.maxAchievableStrafeVelocity.set(MAX_STRAFE_IN_PER_S);
            c.naturalForwardDeceleration.set(FORWARD_DECEL_IN_PER_S2);
            c.naturalStrafeDeceleration.set(STRAFE_DECEL_IN_PER_S2);
        });
    }

    /**
     * @param hardwareMap the OpMode's hardware map
     * @param config      the algorithm settings from {@link #foresightConfig()}
     * @return a follower for the drive, at pose (0, 0, 0) until the auto sets its start pose
     */
    public static Follower createFollower(HardwareMap hardwareMap, ForesightConfig config) {
        Mecanum drivetrain = new Mecanum(hardwareMap, new MecanumConfig(c -> {
            c.frontLeftName.set(HardwareNames.FRONT_LEFT);
            c.frontRightName.set(HardwareNames.FRONT_RIGHT);
            c.backLeftName.set(HardwareNames.BACK_LEFT);
            c.backRightName.set(HardwareNames.BACK_RIGHT);
            // PLACEHOLDER: the left side runs reversed on most mecanum frames.
            c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
            c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
            c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
            c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
        }));
        PinpointLocalizer localizer = new PinpointLocalizer(hardwareMap, new PinpointConfig(c -> {
            c.name.set(HardwareNames.PINPOINT);
            // PLACEHOLDER: pod directions depend on how the pods are mounted.
            c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
            c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
            c.xPodOffset.set(X_POD_OFFSET_IN);
            c.yPodOffset.set(Y_POD_OFFSET_IN);
        }));
        return new Follower(localizer, drivetrain, new Foresight(config));
    }
}
