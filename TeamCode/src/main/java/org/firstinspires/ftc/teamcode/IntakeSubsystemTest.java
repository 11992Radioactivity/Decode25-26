package org.firstinspires.ftc.teamcode;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.impl.MotorEx;

@TeleOp
public class IntakeSubsystemTest extends NextFTCOpMode {
    public IntakeSubsystemTest() {
        addComponents(
                BulkReadComponent.INSTANCE
                //new SubsystemComponent(Intake.INSTANCE)
        );
    }

    JoinedTelemetry joinedTelemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
    ElapsedTime timer;
    // far goal = 3400, close = 2400
    // rpm = 0.0718987(dist^2) + 0.326309(dist) + 2325.5
    double target = 2800;

    @Override
    public void onStartButtonPressed() {
        timer = new ElapsedTime();

        (new MotorEx("FlyWheelL")).setPower(-1);
    }

    @Override
    public void onUpdate() {
        //joinedTelemetry.addData("Shooter target", target);
        //joinedTelemetry.addData("Intake RPM", Intake.INSTANCE.getSpeed());
        joinedTelemetry.update();
    }

    @Override
    public void onStop() {
        //Shooter.INSTANCE.off.schedule();
    }
}
