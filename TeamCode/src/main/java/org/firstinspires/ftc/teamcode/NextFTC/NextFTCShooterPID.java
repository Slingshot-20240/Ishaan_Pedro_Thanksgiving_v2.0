package org.firstinspires.ftc.teamcode.NextFTC;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.Shooternf;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.control.feedforward.BasicFeedforwardParameters;
import dev.nextftc.hardware.controllable.MotorGroup;
import dev.nextftc.hardware.impl.MotorEx;

@Configurable
@TeleOp
public class NextFTCShooterPID extends OpMode {

    MotorEx outtake1, outtake2;
    MotorGroup shooter;

    public static double p = 0.0005, i = 0.0, d = 0.0;
    public static double kV = 0.0009, kA = 0.0, kS = 0.0;
    public static int shooterVel = 1100;

    ControlSystem shooterController;

    @Override
    public void init() {


        outtake1 = new MotorEx("outtake1");
        outtake2 = new MotorEx("outtake2");
        outtake2.reverse();

        shooter = new MotorGroup(outtake1, outtake2);

        shooterController = ControlSystem.builder()
                .velPid(new PIDCoefficients(p, i, d))
                .basicFF(new BasicFeedforwardParameters(kV, kA, kS))
                .build();
    }

    @Override
    public void loop() {

        shooterController.setGoal(new KineticState(0.0, shooterVel));

        double power = shooterController.calculate(shooter.getState());
        shooter.setPower(power);

        telemetry.addData("Target Velocity", shooterVel);
        telemetry.addData("Current Velocity", shooter.getVelocity());
        telemetry.addData("Power", power);
        telemetry.update();
    }
}


