// Copyright (c) 2026 Horizon (FTC 36596). MIT licence: see LICENSE at the repository root.

package org.firstinspires.ftc.teamcode.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.seattlesolvers.solverslib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.testing.FakeDevice;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

class IntakeSubsystemTest {

    private final DcMotorEx motor = FakeDevice.motor();
    private final IntakeSubsystem intake = new IntakeSubsystem(motor);

    @AfterEach
    void resetScheduler() {
        CommandScheduler.getInstance().reset();
    }

    @Test
    void frontAndBothRunTheFrontIntake() {
        intake.set(IntakeSubsystem.Side.FRONT, IntakeSubsystem.State.FORWARD);
        assertEquals(IntakeSubsystem.FORWARD_POWER, FakeDevice.last(motor, "Power"));

        intake.set(IntakeSubsystem.Side.BOTH, IntakeSubsystem.State.REVERSE);
        assertEquals(IntakeSubsystem.REVERSE_POWER, FakeDevice.last(motor, "Power"));

        intake.set(IntakeSubsystem.Side.BOTH, IntakeSubsystem.State.STOP);
        assertEquals(0.0, FakeDevice.last(motor, "Power"));
    }

    @Test
    void backDoesNothingBecauseThereIsNoBackIntake() {
        intake.set(IntakeSubsystem.Side.FRONT, IntakeSubsystem.State.FORWARD);
        intake.set(IntakeSubsystem.Side.BACK, IntakeSubsystem.State.STOP);

        assertEquals(IntakeSubsystem.State.FORWARD, intake.frontState());
        assertEquals(IntakeSubsystem.FORWARD_POWER, FakeDevice.last(motor, "Power"));
    }
}
