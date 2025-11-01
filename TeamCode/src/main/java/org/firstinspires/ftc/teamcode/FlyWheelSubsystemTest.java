package org.firstinspires.ftc.teamcode;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Shooter;

import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.components.BulkReadComponent;

@TeleOp
public class FlyWheelSubsystemTest extends NextFTCTeleop {
    public FlyWheelSubsystemTest() {
        addComponents(
                BulkReadComponent.INSTANCE,
                new SubsystemComponent(Shooter.INSTANCE)
        );
    }

    JoinedTelemetry joinedTelemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());

    @Override
    public void onStartButtonPressed() {
        Shooter.INSTANCE.on.schedule();
    }

    @Override
    public void onUpdate() {
        joinedTelemetry.addData("Shooter RPM", Shooter.INSTANCE.getCurrentSpeed());
        joinedTelemetry.update();
    }
}
