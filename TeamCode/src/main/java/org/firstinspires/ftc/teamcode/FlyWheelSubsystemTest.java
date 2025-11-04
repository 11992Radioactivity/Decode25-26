package org.firstinspires.ftc.teamcode;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Shooter;

import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@TeleOp
public class FlyWheelSubsystemTest extends NextFTCOpMode {
    public FlyWheelSubsystemTest() {
        addComponents(
                BulkReadComponent.INSTANCE,
                new SubsystemComponent(Shooter.INSTANCE)
        );
    }

    JoinedTelemetry joinedTelemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
    ElapsedTime timer;
    double target = 3200;

    @Override
    public void onStartButtonPressed() {
        timer = new ElapsedTime();

        Shooter.INSTANCE.setSpeed(target);
    }

    @Override
    public void onUpdate() {
        if (timer.seconds() > 5) {
            timer.reset();
            //target = (5000 * Math.random()) + 1000;
            Shooter.INSTANCE.setSpeed(target);
        }

        joinedTelemetry.addData("Shooter target", target);
        joinedTelemetry.addData("Shooter RPM", Shooter.INSTANCE.getCurrentSpeed());
        joinedTelemetry.update();
    }

    @Override
    public void onStop() {
        Shooter.INSTANCE.off.schedule();
    }
}
