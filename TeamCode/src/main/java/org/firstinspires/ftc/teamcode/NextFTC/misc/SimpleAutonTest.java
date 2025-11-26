package org.firstinspires.ftc.teamcode.NextFTC.misc;

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

@Autonomous(name = "NextFTC Autonomous Program Java")
public class SimpleAutonTest extends NextFTCOpMode {
    public SimpleAutonTest() {
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
    public PathChain prepareSet2;
    public PathChain grabSet2;
    public PathChain prepareGate;
    public PathChain hitGate;
    public PathChain scoreSet2;
    public PathChain prepareSet3;
    public PathChain grabSet3;
    public PathChain scoreSet3;
    public PathChain prepareSet4;
    public PathChain grabSet4;
    public PathChain scoreSet4;

    public void buildPaths() {
        PedroComponent.follower().setStartingPose(new Pose(126, 118, Math.toRadians(36)));

        scorePreloads = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(126, 118.300), new Pose(97,97))
                )
                .setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(45))
                .build();

        prepareSet2 = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(97,97), new Pose(96.500, 83.800))
                )
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                .build();

        grabSet2 = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(96.500, 83.800), new Pose(124, 83.800))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        prepareGate = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(124, 83.800), new Pose(123.000, 70.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                .build();

        hitGate = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(123.000, 70.000), new Pose(125, 70.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
                .build();

        scoreSet2 = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(125, 70.000), new Pose(97,97))
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(45))
                .build();

        prepareSet3 = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(97,97), new Pose(96.500, 58.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                .build();

        grabSet3 = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(96.500, 58), new Pose(131, 59.5))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        scoreSet3 = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(131, 59.5),
                                new Pose(91.262, 56.240),
                                new Pose(95.176, 82.815),
                                new Pose(97,97)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                .build();

        prepareSet4 = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(97,97), new Pose(96.000, 38))
                )
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                .build();

        grabSet4 = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(96.000, 38), new Pose(130, 37))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        scoreSet4 = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(130, 37), new Pose(90.000, 110.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(29))
                .build();

    }
    private Command autonomous() {
        return new ParallelGroup(
            //INTAKE ALWAYS ON
            Intakenf.INSTANCE.in(),

            //MAIN SEQUENCE
            new SequentialGroup(

                new ParallelGroup(
                        new FollowPath(scorePreloads),

                        Transfernf.INSTANCE.hotdog(),
                        Hoodnf.INSTANCE.closeSide(),
                        Shooternf.INSTANCE.closeSide()
                ),
                //Spin up time
                new Delay(1),
                new ParallelRaceGroup(
                        Transfernf.INSTANCE.on(),
                        new Delay(2.5)
                ),



            //SET 2
                new ParallelGroup(
                        new FollowPath(grabSet2),
                        Transfernf.INSTANCE.hotdog(),
                        //SET 2 Shooter vel
                        Shooternf.INSTANCE.setShooterVel(-1100)
                ),
                new SequentialGroup(
                        new FollowPath(prepareGate),
                        new FollowPath(hitGate),
                        new Delay(1.5),
                        new FollowPath(scoreSet2)
                ),
                new ParallelRaceGroup(
                        Transfernf.INSTANCE.on(),
                        new Delay(2.5)
                ),


            //SET 3
                new ParallelGroup(
                        new SequentialGroup(
                                new FollowPath(grabSet3),
                                new FollowPath(scoreSet3)
                        ),
                        Transfernf.INSTANCE.hotdog(),
                        //SET 3 Shooter vel
                        Shooternf.INSTANCE.setShooterVel(-1100)
                ),

                new ParallelRaceGroup(
                        Transfernf.INSTANCE.on(),
                        new Delay(2.5)
                ),

                //SET 4
                new ParallelGroup(
                        new SequentialGroup(
                                new FollowPath(grabSet4),
                                new FollowPath(scoreSet4)
                        ),
                        Transfernf.INSTANCE.hotdog(),
                        //SET 4 Shooter vel
                        Shooternf.INSTANCE.setShooterVel(-1100)
                ),

                new ParallelRaceGroup(
                        Transfernf.INSTANCE.on(),
                        new Delay(2.5)
                )


            )
        );
    }


    @Override
    public void onInit() {
        buildPaths();
    }
    @Override
    public void onStartButtonPressed() {
        autonomous().schedule();
    }
}