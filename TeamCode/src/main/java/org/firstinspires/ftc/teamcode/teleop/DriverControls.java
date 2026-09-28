// Copyright (c) 2026 Horizon (FTC 36596). MIT licence: see LICENSE at the repository root.

package org.firstinspires.ftc.teamcode.teleop;

import com.pedropathing.drivetrain.DrivePowers;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.commands.ShootAllCommand;
import org.firstinspires.ftc.teamcode.subsystems.HopperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.horizon36596.zenith.RobotClock;

/**
 * Every driver control in one place. Change a button here and nowhere else; the README's
 * "Driving it" table lists the same controls.
 *
 * <pre>
 *   left stick         drive: forward, back and strafe (robot-centric)
 *   right stick x      turn
 *   right bumper held  intake in; it stops by itself when the hopper is full
 *   left bumper held   intake out, to clear a jam
 *   A                  launch one pollen
 *   Y                  launch all four
 *   B                  launcher to idle, ending a volley
 * </pre>
 */
public final class DriverControls {

    /** PLACEHOLDER: stick travel ignored around the centre, so a worn stick does not creep. */
    static final double DEADBAND = 0.05;

    private DriverControls() {}

    /**
     * Binds the buttons. The SolversLib scheduler polls them once per loop.
     *
     * @param driver   gamepad 1
     * @param clock    the clock the launch commands time themselves on
     */
    public static void bind(GamepadEx driver, final IntakeSubsystem intake, final LauncherSubsystem launcher,
            final HopperSubsystem hopper, final RobotClock clock) {
        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(intake.setCommand(IntakeSubsystem.Side.FRONT, IntakeSubsystem.State.FORWARD))
                .whenReleased(intake.setCommand(IntakeSubsystem.Side.FRONT, IntakeSubsystem.State.STOP));
        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(intake.setCommand(IntakeSubsystem.Side.FRONT, IntakeSubsystem.State.REVERSE))
                .whenReleased(intake.setCommand(IntakeSubsystem.Side.FRONT, IntakeSubsystem.State.STOP));
        new Trigger(hopper::isFull)
                .whenActive(intake.setCommand(IntakeSubsystem.Side.FRONT, IntakeSubsystem.State.STOP));

        driver.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(() -> new ShootAllCommand(launcher, hopper, 1, ShootAllCommand.Cadence.PRECISE, clock).schedule());
        driver.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(() -> new ShootAllCommand(launcher, hopper, HopperSubsystem.CAPACITY,
                        ShootAllCommand.Cadence.RAPID, clock).schedule());
        // Requiring the launcher interrupts a volley still running.
        driver.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new InstantCommand(launcher::idle, launcher));
    }

    /** @return the drive powers for the sticks: forward, to the robot's left, counter-clockwise */
    public static DrivePowers drive(GamepadEx driver) {
        return mix(driver.getLeftY(), driver.getLeftX(), driver.getRightX());
    }

    /**
     * Stick values to robot-centric drive powers. Stick right is strafe right and turn clockwise, so
     * both change sign on the way to Pedro's left-positive, counter-clockwise-positive powers.
     *
     * @param leftY  forward on the left stick, -1 to 1 (GamepadEx already makes up positive)
     * @param leftX  right on the left stick, -1 to 1
     * @param rightX right on the right stick, -1 to 1
     */
    static DrivePowers mix(double leftY, double leftX, double rightX) {
        return new DrivePowers(deadband(leftY), -deadband(leftX), -deadband(rightX));
    }

    static double deadband(double value) {
        return Math.abs(value) < DEADBAND ? 0 : value;
    }
}
