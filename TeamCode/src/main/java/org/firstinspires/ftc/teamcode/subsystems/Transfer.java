package org.firstinspires.ftc.teamcode.subsystems;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.CRServoEx;
import dev.nextftc.hardware.powerable.SetPower;

public class Transfer implements Subsystem {
    public static final Transfer INSTANCE = new Transfer();
    private Transfer() {}

    private CRServoEx servo = new CRServoEx("TransferServo");

    public Command on = new SetPower(servo, 1).requires(this).named("TransferOn");
    public Command off = new SetPower(servo, 0).requires(this).named("TransferOff");
}
