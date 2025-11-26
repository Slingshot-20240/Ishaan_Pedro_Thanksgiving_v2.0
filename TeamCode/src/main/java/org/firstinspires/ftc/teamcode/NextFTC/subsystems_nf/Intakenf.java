package org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;

import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;

public class Intakenf implements Subsystem {
    public static final Intakenf INSTANCE = new Intakenf();
    private Intakenf() { }

    public MotorEx intake;

    private enum intakeStates {
        IN (1.0),
        IDLE (0),
        OUT (-1.0);

        private final double intakeState;
        intakeStates(double state) {
            this.intakeState = state;
        }
        public double getState() {
            return intakeState;
        }
    }

    public Command in() {
        return new SetPower(intake, Intakenf.intakeStates.IN.getState());
    }
    public Command idle() {
        return new SetPower(intake, Intakenf.intakeStates.IDLE.getState());
    }
    public Command out() {
        return new SetPower(intake, Intakenf.intakeStates.OUT.getState());
    }

    @Override
    public void initialize() {
        intake = new MotorEx("intake");
    }

    @Override
    public void periodic() {}
}
