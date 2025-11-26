package org.firstinspires.ftc.teamcode.NextFTC.autonomous;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.Intakenf;
import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.Shooternf;
import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.Transfernf;
import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.Hoodnf;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.bindings.Variable;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.ParallelRaceGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Autonomous(name = "SplineAutonTest")
public class SplineAutonTest extends NextFTCOpMode {
    public SplineAutonTest() {
        addComponents(
                new SubsystemComponent(
                        Intakenf.INSTANCE, Hoodnf.INSTANCE,
                        Shooternf.INSTANCE, Transfernf.INSTANCE
                ),
                new PedroComponent(Constants::createFollower),
                BulkReadComponent.INSTANCE
        );
    }
    public PathChain scorePreloads;
    public PathChain grabSet2;
    public PathChain hitGate;
    public PathChain scoreSet2;
    public PathChain grabSet3;
    public PathChain scoreSet3;
    public PathChain turnSet4;
    public PathChain driveToSet4;
    public PathChain turnToBallsSet4;
    public PathChain grabSet4;
    public PathChain scoreSet4;


    public void buildPaths() {
        PedroComponent.follower().setStartingPose(new Pose(126, 118, Math.toRadians(36)));

        scorePreloads = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(126.200, 119.000), new Pose(87.000, 87.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(45))
                .build();

        grabSet2 = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(87.000, 87.000),
                                new Pose(92.292, 80.549),
                                new Pose(129.000, 83.800)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                .build();

        hitGate = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(129.000, 83.800),
                                new Pose(113.000, 77.000),
                                new Pose(130.000, 71.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                .build();

        scoreSet2 = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(130.000, 71.000), new Pose(87.000, 87.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(45))
                .build();

        grabSet3 = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(87.000, 87.000),
                                new Pose(87.760, 65.305),
                                new Pose(79.313, 56.446),
                                new Pose(136.172, 58.094)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                .build();

        scoreSet3 = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(136.172, 58.094),
                                new Pose(91.262, 56.240),
                                new Pose(95.176, 82.815),
                                new Pose(96.500, 96.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                .build();

        turnSet4 = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(96.500, 96.000), new Pose(96.500, 96.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(270))
                .build();

        driveToSet4 = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(96.500, 96.000), new Pose(96.500, 36.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(270))
                .build();

        turnToBallsSet4 = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(96.500, 36.000), new Pose(96.500, 36.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(0))
                .build();

        grabSet4 = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(96.500, 36.000), new Pose(137.000, 36.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        scoreSet4 = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(137.000, 36.000), new Pose(90.000, 110.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(36))
                .build();

    }

    private Command init_bot() {
        return new SequentialGroup(
                Hoodnf.INSTANCE.setHoodPos(0.5)
        );
    }

    private Command transferUpFor(double time) {
        return new ParallelRaceGroup(
                Transfernf.INSTANCE.on(),
                new Delay(time)
        );
    }

    private Command baseState() {
        return new ParallelGroup(
                Transfernf.INSTANCE.hotdog(),
                Hoodnf.INSTANCE.setHoodPos(0.5)
        );
    }

    private Command autonomous() {
        return new ParallelGroup(
                //INTAKE ALWAYS ON
                Intakenf.INSTANCE.in(),

                //MAIN SEQUENCE
                new SequentialGroup(

                        new ParallelGroup(
                                new FollowPath(scorePreloads),

                                baseState(),
                                Shooternf.INSTANCE.setShooterVel(-1200)
                        ),
                        //Spin up time
                        new Delay(1),
                        transferUpFor(2.5),



                        //SET 2
                        new ParallelGroup(
                                new FollowPath(grabSet2),

                                baseState(),
                                //SET 2 Shooter vel
                                Shooternf.INSTANCE.setShooterVel(-1200)
                        ),
                        new SequentialGroup(
                                new FollowPath(hitGate),
                                new Delay(1.5),
                                new FollowPath(scoreSet2)
                        ),
                        transferUpFor(2.5)

//
//                        //SET 3
//                        new ParallelGroup(
//                                new SequentialGroup(
//                                        new FollowPath(grabSet3),
//                                        new FollowPath(scoreSet3)
//                                ),
//                                Transfernf.INSTANCE.hotdog(),
//                                //SET 3 Shooter vel
//                                Shooternf.INSTANCE.setShooterVel(-1100)
//                        ),
//
//                        new ParallelRaceGroup(
//                                Transfernf.INSTANCE.on(),
//                                new Delay(2.5)
//                        ),
//
//                        //SET 4
//                        new ParallelGroup(
//                                new SequentialGroup(
//                                        new FollowPath(grabSet4),
//                                        new FollowPath(scoreSet4)
//                                ),
//                                Transfernf.INSTANCE.hotdog(),
//                                //SET 4 Shooter vel
//                                Shooternf.INSTANCE.setShooterVel(-1100)
//                        ),
//
//                        new ParallelRaceGroup(
//                                Transfernf.INSTANCE.on(),
//                                new Delay(2.5)
//                        )


                )
        );
    }


    @Override
    public void onInit() {
        buildPaths();
        init_bot().schedule();
    }

    @Override
    public void onStartButtonPressed() {
        autonomous().schedule();
    }
}