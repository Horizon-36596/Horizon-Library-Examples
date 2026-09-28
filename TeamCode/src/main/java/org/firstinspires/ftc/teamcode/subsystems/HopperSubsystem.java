// Copyright (c) 2026 Horizon (FTC 36596). MIT licence: see LICENSE at the repository root.

package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.hardware.HardwareNames;

/** The hopper between the intake and the launcher. It holds four pollen, one per slot. */
public class HopperSubsystem extends SubsystemBase {

    public static final int CAPACITY = 4;

    private final DigitalChannel[] slots;

    /** @param slots one beam-break sensor per slot; a broken beam means the slot holds a pollen */
    public HopperSubsystem(DigitalChannel... slots) {
        if (slots.length != CAPACITY) {
            throw new IllegalArgumentException("the hopper has " + CAPACITY + " slots, got " + slots.length);
        }
        this.slots = slots.clone();
        for (DigitalChannel slot : this.slots) {
            slot.setMode(DigitalChannel.Mode.INPUT);
        }
    }

    public static HopperSubsystem create(HardwareMap hardwareMap) {
        return new HopperSubsystem(
                hardwareMap.get(DigitalChannel.class, HardwareNames.HOPPER_SLOT_1),
                hardwareMap.get(DigitalChannel.class, HardwareNames.HOPPER_SLOT_2),
                hardwareMap.get(DigitalChannel.class, HardwareNames.HOPPER_SLOT_3),
                hardwareMap.get(DigitalChannel.class, HardwareNames.HOPPER_SLOT_4));
    }

    /** @return how many slots hold a pollen */
    public int count() {
        int held = 0;
        for (DigitalChannel slot : slots) {
            // A beam-break sensor reads high while its beam is unbroken.
            if (!slot.getState()) {
                held++;
            }
        }
        return held;
    }

    public boolean isFull() {
        return count() == CAPACITY;
    }

    public boolean isEmpty() {
        return count() == 0;
    }
}
