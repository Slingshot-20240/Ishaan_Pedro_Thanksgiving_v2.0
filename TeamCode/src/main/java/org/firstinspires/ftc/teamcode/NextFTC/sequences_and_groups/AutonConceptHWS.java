package org.firstinspires.ftc.teamcode.NextFTC.sequences_and_groups;



import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.Intakenf;
import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.Shooternf;
import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.Transfernf;
import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.Hoodnf;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.subsystems.SubsystemGroup;

public class AutonConceptHWS extends SubsystemGroup {
    public static final AutonConceptHWS INSTANCE = new AutonConceptHWS();

    private AutonConceptHWS() {
        super(
                AutonConceptHWS.INSTANCE,
                Intakenf.INSTANCE, Transfernf.INSTANCE,
                Shooternf.INSTANCE, Hoodnf.INSTANCE
        );
    }


    public Command intakeInFor(double time) {
        return new SequentialGroup(
                Intakenf.INSTANCE.in(),
                new Delay(time),
                Intakenf.INSTANCE.idle()
        );
    }

    public Command transferUpFor(double time) {
        return new SequentialGroup(
                Transfernf.INSTANCE.on(),
                new Delay(time),
                Transfernf.INSTANCE.hotdog()
        );
    }

    public Command shootFor(double time, double vel) {
        return new SequentialGroup(
                Shooternf.INSTANCE.setShooterVel(vel),
                new Delay(time),
                Intakenf.INSTANCE.idle()
        );
    }



}