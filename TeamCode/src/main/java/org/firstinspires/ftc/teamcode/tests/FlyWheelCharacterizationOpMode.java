package org.firstinspires.ftc.teamcode.tests;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.controllable.MotorGroup;
import dev.nextftc.hardware.impl.MotorEx;

@TeleOp
public class FlyWheelCharacterizationOpMode extends NextFTCOpMode {
    public FlyWheelCharacterizationOpMode() {
        addComponents(
                BulkReadComponent.INSTANCE
        );
    }

    JoinedTelemetry joinedTelemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
    private MotorGroup motors = new MotorGroup(
            (new MotorEx("FlyWheelR")).floatMode(), // right is leader because it doesn't have to be reversed
            (new MotorEx("FlyWheelL")).reversed().floatMode()
    );
    ElapsedTime timer;
    ElapsedTime loopTimer;
    VoltageSensor voltageSensor;

    double runtime = 30;
    double volts = 0;
    double voltRate = 12 / runtime;
    double velSum = 0;
    double voltSum = 0;
    double velSqrSum = 0;
    double voltVelSum = 0;
    double count = 0;

    @Override
    public void onStartButtonPressed(){
        timer = new ElapsedTime();
        loopTimer = new ElapsedTime();
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
    }

    @Override
    public void onUpdate() {
        double dt = loopTimer.seconds();
        loopTimer.reset();

        double vel = motors.getVelocity();

        // only collect data up until time is up
        if (timer.seconds() < runtime) {
            volts += voltRate * dt;

            // only start collecting once motor moves faster than ~10 rpm
            if (vel > 5) {
                velSum += vel;
                voltSum += volts;
                velSqrSum += vel * vel;
                voltVelSum += volts * vel;
                count++;
            }

            motors.setPower(volts / voltageSensor.getVoltage());
        } else {
            motors.setPower(0);
            volts = 0;
        }

        double kv = (count * voltVelSum - velSum * voltSum) / (count * velSqrSum - velSum * velSum);
        double ks = (voltSum / count) - (kv * velSum) / count;

        joinedTelemetry.addData("voltage", volts);
        joinedTelemetry.addData("velocity (ticks/sec)", vel);
        joinedTelemetry.addData("kv (divide by 12 to make for power in (-1, 1) rather than volts)", kv);
        joinedTelemetry.addData("ks (divide by 12 for same reason)", ks);
        joinedTelemetry.update();
    }
}
