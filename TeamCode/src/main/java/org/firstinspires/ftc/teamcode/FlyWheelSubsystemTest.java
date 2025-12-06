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
    // far goal = 3400, close = 2400, tip of close = 2800
    // rpm = 0.0718987(dist^2) + 0.326309(dist) + 2325.5
    double min_speed = 2000;
    double max_speed = 3200;

    @Override
    public void onStartButtonPressed() {
        timer = new ElapsedTime();

        Shooter.INSTANCE.setSpeed(min_speed);
    }

    @Override
    public void onUpdate() {
        if (timer.seconds() > 2.5) {
            Shooter.INSTANCE.setSpeed(max_speed);
        }
        if (timer.seconds() > 5) {
            Shooter.INSTANCE.setSpeed(min_speed);
            timer.reset();
        }

        joinedTelemetry.addData("Shooter target", Shooter.INSTANCE.getTargetSpeed());
        joinedTelemetry.addData("Shooter current", Shooter.INSTANCE.getCurrentSpeed());
        joinedTelemetry.addData("timer", timer.seconds());
        joinedTelemetry.update();
    }

    @Override
    public void onStop() {
        Shooter.INSTANCE.off.schedule();
    }
}
