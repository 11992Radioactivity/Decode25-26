package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.psilynx.psikit.core.rlog.RLOGServer;
import org.psilynx.psikit.core.rlog.RLOGWriter;
import org.psilynx.psikit.core.Logger;

import org.psilynx.psikit.ftc.PsiKitOpMode;

@TeleOp
@Disabled
public class PsiKitTest extends PsiKitOpMode {
    @Override
    public void psiKit_init() {
        Logger.addDataReceiver(new RLOGServer());
        Logger.addDataReceiver(new RLOGWriter("log.rlog"));

        Logger.recordMetadata("some metadata", "string value");
        //examples
    }
    public void psiKit_init_loop() {
        /*

         init loop logic goes here

        */
    }
    @Override
    public void psiKit_start() {
        // start logic here
    }
    @Override
    public void psiKit_loop() {

        /*

         OpMode logic goes here

        */

        Logger.recordOutput("OpMode/example", 2.0);
        // example

    }

    @Override
    public void psiKit_stop() {
        // end logic goes here
    }
}