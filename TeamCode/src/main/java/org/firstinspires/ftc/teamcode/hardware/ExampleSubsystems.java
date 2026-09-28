// Copyright (c) 2026 Horizon (FTC 36596). MIT licence: see LICENSE at the repository root.

package org.firstinspires.ftc.teamcode.hardware;

import com.pedropathing.algorithm.ForesightConfig;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.HopperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;

/** The robot's four subsystems, built once from the hardware map. The autos and the TeleOp share it. */
public final class ExampleSubsystems {

    public final DriveSubsystem drive;
    public final IntakeSubsystem intake;
    public final LauncherSubsystem launcher;
    public final HopperSubsystem hopper;

    private ExampleSubsystems(DriveSubsystem drive, IntakeSubsystem intake, LauncherSubsystem launcher,
            HopperSubsystem hopper) {
        this.drive = drive;
        this.intake = intake;
        this.launcher = launcher;
        this.hopper = hopper;
    }

    public static ExampleSubsystems create(HardwareMap hardwareMap) {
        ForesightConfig foresight = PedroSetup.foresightConfig();
        return new ExampleSubsystems(
                new DriveSubsystem(PedroSetup.createFollower(hardwareMap, foresight), foresight),
                IntakeSubsystem.create(hardwareMap),
                LauncherSubsystem.create(hardwareMap),
                HopperSubsystem.create(hardwareMap));
    }
}
