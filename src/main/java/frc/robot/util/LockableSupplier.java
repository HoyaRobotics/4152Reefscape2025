// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.function.Supplier;

/** Add your docs here. */
public class LockableSupplier<T> implements Supplier<T> {
    private final Supplier<T> supplier;
    private boolean locked = false;
    private T lastValue = null;

    public LockableSupplier(Supplier<T> supplier) {
        this.supplier = supplier;
    }

    @Override
    public T get() {
        return locked ? lastValue : supplier.get();
    }

    public void lock() {
        locked = true;
        lastValue = supplier.get();
    }

    public void unlock() {
        locked = false;
    }
}
