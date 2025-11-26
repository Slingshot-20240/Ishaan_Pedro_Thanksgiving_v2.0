package org.firstinspires.ftc.teamcode.NextFTC.sequences_and_groups;



import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.Intakenf;
import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.Shooternf;
import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.Transfernf;
import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.Hoodnf;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.subsystems.SubsystemGroup;

public class AutonConceptSequences extends SubsystemGroup {
    public static final AutonConceptSequences INSTANCE = new AutonConceptSequences();

    private AutonConceptSequences() {
        super(
                AutonConceptSequences.INSTANCE,
                AutonConceptHWS.INSTANCE,
                Intakenf.INSTANCE, Transfernf.INSTANCE,
                Shooternf.INSTANCE, Hoodnf.INSTANCE
        );
    }

    /**
     * Runs the Intake
     * Then waits for @speedUpTime
     * Then transfers for @transferTime
     */
    public final Command scoreSet(double speedUpTime, double transferTime) {
        return new SequentialGroup(
                Intakenf.INSTANCE.in(),
                new SequentialGroup(
                        new Delay(speedUpTime),
                        AutonConceptHWS.INSTANCE.transferUpFor(transferTime)
                )
        );
    }

    /**
     * Runs the Intake while hotdogging
     * Sets Shooter Power
     */
    public final Command intakeSet(double shooterPower) {
        return new ParallelGroup(
                Intakenf.INSTANCE.in(),
                Transfernf.INSTANCE.hotdog(),
                Shooternf.INSTANCE.setShooterVel(shooterPower)
        );
    }


}