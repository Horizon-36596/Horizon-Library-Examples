// Copyright (c) 2026 Horizon (FTC 36596). MIT licence: see LICENSE at the repository root.

package org.firstinspires.ftc.teamcode.zenith;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.commands.ShootAllCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.HopperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.horizon36596.zenith.solverslib.NamedCommands;

/**
 * The command and condition names an auto file may use. They are the same names, with the same
 * arguments, as the commands and conditions in autos/robot.json.
 */
public final class ExampleCommands {

    public static final String SHOOT_ALL = "shootAll";
    public static final String SET_INTAKE = "setIntake";
    public static final String LAUNCHER_IDLE = "launcherIdle";
    public static final String RELOCALIZE = "relocalize";
    public static final String CANCEL_ALL = "cancelAll";

    public static final String HOPPER_FULL = "hopperFull";
    public static final String HOPPER_EMPTY = "hopperEmpty";

    private ExampleCommands() {}

    /** Registers every name with the Zenith runtime. Call it from ZenithRobot.registerCommands. */
    public static void register(final DriveSubsystem drive, final IntakeSubsystem intake,
            final LauncherSubsystem launcher, final HopperSubsystem hopper) {
        NamedCommands.register(SHOOT_ALL, (args, ctx) -> new ShootAllCommand(launcher, hopper,
                NamedCommands.argInt(args, SHOOT_ALL, "count"),
                ShootAllCommand.Cadence.fromFile(NamedCommands.argString(args, SHOOT_ALL, "cadence", "precise")),
                ctx));
        NamedCommands.register(SET_INTAKE, (args, ctx) -> intake.setCommand(
                NamedCommands.argEnum(args, SET_INTAKE, "side", IntakeSubsystem.Side.class, IntakeSubsystem.Side.BOTH),
                NamedCommands.argEnum(args, SET_INTAKE, "state", IntakeSubsystem.State.class, IntakeSubsystem.State.STOP)));
        NamedCommands.register(LAUNCHER_IDLE, (args, ctx) -> new InstantCommand(launcher::idle, launcher));
        NamedCommands.register(RELOCALIZE, (args, ctx) -> new InstantCommand(drive::relocalize, drive));
        NamedCommands.register(CANCEL_ALL, (args, ctx) -> new InstantCommand(() -> {
            intake.set(IntakeSubsystem.Side.BOTH, IntakeSubsystem.State.STOP);
            launcher.stop();
        }, intake, launcher, hopper));

        NamedCommands.registerCondition(HOPPER_FULL, hopper::isFull);
        NamedCommands.registerCondition(HOPPER_EMPTY, hopper::isEmpty);
    }
}
