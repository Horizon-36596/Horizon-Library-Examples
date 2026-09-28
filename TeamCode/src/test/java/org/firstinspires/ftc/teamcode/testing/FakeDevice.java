// Copyright (c) 2026 Horizon (FTC 36596). MIT licence: see LICENSE at the repository root.

package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import java.lang.reflect.InvocationHandler;
import java.lang.reflect.Method;
import java.lang.reflect.Proxy;
import java.util.HashMap;
import java.util.Map;

/**
 * A stand-in for an SDK device, for tests on the JVM with no Control Hub. It remembers the last value
 * each {@code setX} was given and returns it from the matching {@code getX}, so a flywheel set to a
 * velocity reports that velocity and a servo reports its position.
 */
public final class FakeDevice implements InvocationHandler {

    private final Map<String, Object> values = new HashMap<String, Object>();

    private FakeDevice() {}

    public static DcMotorEx motor() {
        return create(DcMotorEx.class, new FakeDevice());
    }

    public static Servo servo() {
        return create(Servo.class, new FakeDevice());
    }

    /** @param beamBroken the reading to report: true when the slot holds a pollen */
    public static DigitalChannel beamBreak(boolean beamBroken) {
        FakeDevice device = new FakeDevice();
        device.values.put("State", !beamBroken);
        return create(DigitalChannel.class, device);
    }

    /** @return the value the device was last given for {@code property}, such as "Power" */
    public static Object last(Object device, String property) {
        return ((FakeDevice) Proxy.getInvocationHandler(device)).values.get(property);
    }

    /** Makes a beam-break sensor from {@link #beamBreak} report a new reading. */
    public static void setBeamBroken(Object sensor, boolean beamBroken) {
        ((FakeDevice) Proxy.getInvocationHandler(sensor)).values.put("State", !beamBroken);
    }

    private static <T> T create(Class<T> type, FakeDevice device) {
        return type.cast(Proxy.newProxyInstance(type.getClassLoader(), new Class<?>[] {type}, device));
    }

    @Override
    public Object invoke(Object proxy, Method method, Object[] args) {
        String name = method.getName();
        if (name.startsWith("set") && args != null && args.length == 1) {
            values.put(name.substring(3), args[0]);
            return null;
        }
        if (name.startsWith("get") && (args == null || args.length == 0)) {
            Object value = values.get(name.substring(3));
            if (value != null) {
                return value;
            }
        }
        if (name.equals("toString")) {
            return "FakeDevice" + values;
        }
        return defaultFor(method.getReturnType());
    }

    private static Object defaultFor(Class<?> type) {
        if (type == boolean.class) {
            return false;
        }
        if (type == double.class) {
            return 0.0;
        }
        if (type == int.class) {
            return 0;
        }
        if (type == long.class) {
            return 0L;
        }
        return null;
    }
}
