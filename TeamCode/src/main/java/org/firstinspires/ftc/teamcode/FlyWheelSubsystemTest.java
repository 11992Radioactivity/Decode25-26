package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.DoubleShooter;

import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.components.BulkReadComponent;

@TeleOp
public class FlyWheelSubsystemTest extends NextFTCTeleop {
    public FlyWheelSubsystemTest() {
        addComponents(
                BulkReadComponent.INSTANCE,
                new SubsystemComponent(DoubleShooter.INSTANCE)
        );
    }

    @Override
    public void onStartButtonPressed() {
        DoubleShooter.INSTANCE.on.schedule();
    }
}
