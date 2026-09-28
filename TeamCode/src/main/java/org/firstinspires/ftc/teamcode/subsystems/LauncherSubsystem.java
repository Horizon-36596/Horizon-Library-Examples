// Copyright (c) 2026 Horizon (FTC 36596). MIT licence: see LICENSE at the repository root.

package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.hardware.HardwareNames;

/**
 * The fixed launcher at the back of the robot: one flywheel on velocity control and a servo that
 * feeds one pollen at a time from the hopper into it.
 */
public class LauncherSubsystem extends SubsystemBase {

    /** PLACEHOLDER: flywheel speed for a launch into the up cell, encoder ticks per second. */
    static final double LAUNCH_TICKS_PER_S = 1800;
    /** PLACEHOLDER: flywheel speed between volleys, so the next spin-up is short. */
    static final double IDLE_TICKS_PER_S = 600;
    /** PLACEHOLDER: how close to launch speed counts as ready. */
    static final double READY_TOLERANCE_TICKS_PER_S = 60;
    /** PLACEHOLDER: feeder servo positions. */
    static final double FEEDER_REST = 0.30;
    static final double FEEDER_PUSH = 0.65;

    private final DcMotorEx flywheel;
    private final Servo feeder;
    private double targetTicksPerS;

    public LauncherSubsystem(DcMotorEx flywheel, Servo feeder) {
        this.flywheel = flywheel;
        this.feeder = feeder;
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        feeder.setPosition(FEEDER_REST);
    }

    public static LauncherSubsystem create(HardwareMap hardwareMap) {
        return new LauncherSubsystem(
                hardwareMap.get(DcMotorEx.class, HardwareNames.LAUNCHER),
                hardwareMap.get(Servo.class, HardwareNames.FEEDER));
    }

    public void spinUp() {
        setTarget(LAUNCH_TICKS_PER_S);
    }

    public void idle() {
        setTarget(IDLE_TICKS_PER_S);
    }

    public void stop() {
        setTarget(0);
        feeder.setPosition(FEEDER_REST);
    }

    /** @return true when the flywheel is at launch speed */
    public boolean ready() {
        return targetTicksPerS == LAUNCH_TICKS_PER_S
                && Math.abs(flywheel.getVelocity() - LAUNCH_TICKS_PER_S) <= READY_TOLERANCE_TICKS_PER_S;
    }

    /** Pushes one pollen into the flywheel, or pulls the feeder back to take the next one. */
    public void feed(boolean push) {
        feeder.setPosition(push ? FEEDER_PUSH : FEEDER_REST);
    }

    public double targetTicksPerS() {
        return targetTicksPerS;
    }

    private void setTarget(double ticksPerS) {
        targetTicksPerS = ticksPerS;
        flywheel.setVelocity(ticksPerS);
    }
}
