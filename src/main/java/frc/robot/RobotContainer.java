// Copyright (c) FIRST and other WPILib contributors.
// eOpen Source Software; you can modify and/or share it under the terms of
// th WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AutoAlgaeGrabCommand;
import frc.robot.commands.AutoAlgaeScoreCommand;
import frc.robot.commands.AutoGameCommand;
import frc.robot.commands.AutoGamePrepCommand;
import frc.robot.commands.AutonomousFeedTillFirstLidar;
import frc.robot.commands.BargeScoreCommand;
import frc.robot.commands.FeederManipulatorCommand;
import frc.robot.commands.GoToArmevatorPosAndGrip;
import frc.robot.constants.TunerConstants;
import frc.robot.constants.positions.ArmevatorPositions.ArmevatorPosition;
import frc.robot.subsystems.algaeManipulator.AlgaeManipulator;
import frc.robot.subsystems.algaeManipulator.states.AlgaeIdle;
import frc.robot.subsystems.algaeManipulator.states.AlgaeIntake;
import frc.robot.subsystems.algaeManipulator.states.AlgaeOuttake;
import frc.robot.subsystems.armevator.Armevator;
import frc.robot.subsystems.armevator.states.GoToArmevatorPoseState;
import frc.robot.subsystems.armevator.states.GoToNextArmevatorPoseState;
import frc.robot.subsystems.armevator.states.ZeroState;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.states.LowerState;
import frc.robot.subsystems.climber.states.ClimbState;
import frc.robot.subsystems.coralManipulator.CoralManipulator;
import frc.robot.subsystems.coralManipulator.states.CoralOutakeState;
import frc.robot.subsystems.coralManipulator.states.IdleState;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.states.DriveState;
import frc.robot.subsystems.drivetrain.states.ResetHeadingState;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.states.FeedState;
import frc.robot.utils.controllers.ButtonBoard;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static frc.robot.constants.SubsystemConstants.DrivetrainConstants.TeleConstants.MAX_TRANSLATION;
import static frc.robot.constants.FieldConstants.*;
import static frc.robot.constants.positions.ArmevatorPositions.*;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

public class RobotContainer {
    private boolean elliott = false; //is elliot driving? //yes
    private boolean buttonBoardInUse = true; // Is an override tool if the button board does not work.

    private ShuffleboardTab autoTab;
    private SendableChooser<Command> autoChooser;

    private final Telemetry logger = new Telemetry(MAX_TRANSLATION);

    private final Joystick flightstick = new Joystick(0);
    private final CommandXboxController driverController = new CommandXboxController(4);
    private final ButtonBoard buttonBoard = new ButtonBoard(1, 2);
    private final CommandXboxController operatorController = new CommandXboxController(3);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public static final CoralManipulator coralManipulator = new CoralManipulator();
    public static final Armevator armevator = new Armevator(coralManipulator.getArmEncoder());
    public static final AlgaeManipulator algaeManipulator = new AlgaeManipulator();
    public static final Feeder feeder = new Feeder();
    public static final Climber climber = new Climber();

    public RobotContainer() {
        setDefaultCommands();
        configureBindings();

        if(buttonBoardInUse) {
            configureButtons();
        } else {
            configureBackupBindings();
        }

        setupAutos();
    }

    private void setDefaultCommands() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        if(elliott) {
            drivetrain.setDefaultCommand(
                new DriveState(
                    drivetrain, 
                    () -> Math.pow(flightstick.getY(), 3),
                    () -> Math.pow(flightstick.getX(), 3),
                    () -> Math.pow(flightstick.getZ(), 3)
                )
            );
        } else {
            drivetrain.setDefaultCommand(
                new DriveState(
                    drivetrain, 
                    flightstick::getY,
                    flightstick::getX,
                    flightstick::getZ,
                )
            ); // Andy code cause andy built like a beast with linear.
        }

        armevator.setDefaultCommand(
            new GoToArmevatorPoseState(
                armevator, 
                new ArmevatorPosition(Rotation2d.fromDegrees(10), Meters.convertFrom(0.1, Inches))
            ).repeatedly()
        );

        coralManipulator.setDefaultCommand(
            new IdleState(coralManipulator, armevator::getArmRotation)
        );

        algaeManipulator.setDefaultCommand(
            new AlgaeIdle(algaeManipulator)
        );
    }
        
    private void configureBindings() {
        // reset the field-centric heading on b press
        driverController.b().onTrue(new ResetHeadingState(drivetrain).ignoringDisable(true));

        driverController.rightStick().whileTrue(
            new AutoAlgaeScoreCommand(
                drivetrain, 
                armevator, 
                algaeManipulator
            )
        );

        driverController.leftTrigger().whileTrue(
            new FeederManipulatorCommand(
                feeder, 
                coralManipulator, 
                armevator
            ).withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        );

        driverController.rightTrigger().whileTrue(
            new FeedState(feeder, -1.0)
                .alongWith(new CoralOutakeState(coralManipulator, 0.5))
        );

        if(!buttonBoardInUse) {
            driverController.rightBumper().whileTrue(
                new ConditionalCommand(
                    new CoralOutakeState(coralManipulator, 0.8), 
                    new ConditionalCommand(
                        new CoralOutakeState(coralManipulator, -0.25), 
                        new CoralOutakeState(coralManipulator, -0.8),
                        armevator::nextIsL1
                    ),
                    armevator::nextIsL4
                )
            );

            driverController.leftBumper().whileTrue(
                new ConditionalCommand(
                    new CoralOutakeState(coralManipulator, -0.8), 
                    new CoralOutakeState(coralManipulator, 0.8), 
                    armevator::nextIsL4
                )
            );

            driverController.x().whileTrue(
                new AutoGameCommand(
                    drivetrain, 
                    armevator, 
                    feeder, 
                    coralManipulator,
                    algaeManipulator,
                    () -> driverController.getHID().getAButtonPressed()
                ).repeatedly().withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
            );

            driverController.y().whileTrue(
                new AutoGameCommand(
                    drivetrain, 
                    armevator, 
                    feeder, 
                    coralManipulator,
                    algaeManipulator,
                    () -> driverController.getHID().getAButtonPressed()
                ).repeatedly().beforeStarting(
                    new AutoGamePrepCommand(
                        drivetrain, 
                        armevator, 
                        feeder, 
                        coralManipulator,
                        algaeManipulator
                    )
                ).withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
            );
        } else {
            driverController.leftBumper().whileTrue(
                new AutoGameCommand(
                    drivetrain, 
                    armevator, 
                    feeder, 
                    coralManipulator,
                    algaeManipulator,
                    () -> driverController.getHID().getAButtonPressed()
                ).repeatedly().withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
            );

            driverController.rightBumper().whileTrue(
                new AutoGameCommand(
                    drivetrain, 
                    armevator, 
                    feeder, 
                    coralManipulator,
                    algaeManipulator,
                    () -> driverController.getHID().getAButtonPressed()
                ).repeatedly().beforeStarting(
                    new AutoGamePrepCommand(
                        drivetrain, 
                        armevator, 
                        feeder, 
                        coralManipulator,
                        algaeManipulator
                    )
                ).withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
            );
        }

        driverController.start().whileTrue(
            new ZeroState(armevator)  
        );

        driverController.back().whileTrue(
            new InstantCommand(() -> armevator.resetArm())
        );

        driverController.x().whileTrue(
            new SequentialCommandGroup(
                new AutoAlgaeGrabCommand(drivetrain, armevator, algaeManipulator),
                new AutoAlgaeScoreCommand(drivetrain, armevator, algaeManipulator)
            ).withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
            // new PathfindingState(drivetrain, drivetrain::getNearestAlgae)
        );

        driverController.povDownLeft().onTrue(
            // new PathfindingState(drivetrain, getGlobalPositions().CORAL_STATION_LEFT)
            new InstantCommand(() -> drivetrain.setNextFeedPose(getGlobalPositions().CORAL_STATION_LEFT_CLOSE_POINT, getGlobalPositions().CORAL_STATION_LEFT_CLOSE)).ignoringDisable(true)
        );
        
        driverController.povUpLeft().whileTrue(
            new InstantCommand(() -> drivetrain.setNextFeedPose(getGlobalPositions().CORAL_STATION_LEFT_FAR_POINT, getGlobalPositions().CORAL_STATION_LEFT_FAR)).ignoringDisable(true)
        );

        driverController.povDownRight().onTrue(
            // new PathfindingState(drivetrain, getGlobalPositions().CORAL_STATION_RIGHT)
            new InstantCommand(() -> drivetrain.setNextFeedPose(getGlobalPositions().CORAL_STATION_RIGHT_CLOSE_POINT, getGlobalPositions().CORAL_STATION_RIGHT_CLOSE)).ignoringDisable(true)
        );

        driverController.povUpRight().onTrue(
            // new PathfindingState(drivetrain, getGlobalPositions().CORAL_STATION_RIGHT)
            new InstantCommand(() -> drivetrain.setNextFeedPose(getGlobalPositions().CORAL_STATION_RIGHT_FAR_POINT, getGlobalPositions().CORAL_STATION_RIGHT_FAR)).ignoringDisable(true)
        );
        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // driverController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // driverController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        if(!Robot.isReal()) {
            drivetrain.registerTelemetry(logger::telemeterize);
        }
    }

    private void configureButtons() {
        buttonBoard.getButton(7).whileTrue(
            new ClimbState(climber)
        );
        buttonBoard.getButton(8).whileTrue(
            new LowerState(climber)
        );

        buttonBoard.getButton(5).whileTrue(
            new ConditionalCommand(
                new CoralOutakeState(coralManipulator, 0.8), 
                new ConditionalCommand(
                    new CoralOutakeState(coralManipulator, -0.25), 
                    new CoralOutakeState(coralManipulator, -0.8),
                    armevator::nextIsL1
                ),
                armevator::nextIsL4
            )
        );

        buttonBoard.getButton(6).whileTrue(
            new ConditionalCommand(
                new CoralOutakeState(coralManipulator, -0.8), 
                new CoralOutakeState(coralManipulator, 0.8),
                armevator::nextIsL4
            )  
        );

        buttonBoard.getButton(4 + 16).whileTrue(
            new SequentialCommandGroup(
                new InstantCommand(() -> drivetrain.setAlgaeGrab(true)),
                new WaitCommand(0.2),
                new GoToArmevatorPoseState(armevator, ALGAE_ARMEVATOR_POSITION)
                    .alongWith(new AlgaeIntake(algaeManipulator)).repeatedly()
            ) 
        );

        buttonBoard.getButton(3 + 16).whileTrue(
            new SequentialCommandGroup(
                new InstantCommand(() -> drivetrain.setAlgaeGrab(true)),
                new WaitCommand(0.2),
                new GoToArmevatorPoseState(armevator, ALGAE_ARMEVATOR_POSITION_TWO)
                    .alongWith(new AlgaeIntake(algaeManipulator)).repeatedly()
            ) 
        );

        buttonBoard.getButton(10).onTrue(
            new InstantCommand(() -> drivetrain.setNextBargePose(getGlobalPositions().MIDDLE_BARGE, getGlobalPositions().MIDDLE_BARGE_PATH)).ignoringDisable(true)
        );

        buttonBoard.getButton(9).onTrue(
            new InstantCommand(() -> drivetrain.setNextBargePose(getGlobalPositions().RIGHT_BARGE, getGlobalPositions().RIGHT_BARGE_PATH)).ignoringDisable(true)  
        );

        buttonBoard.getButton(11).whileTrue(
            new SequentialCommandGroup(
                new InstantCommand(() -> drivetrain.setNextBargePose(getGlobalPositions().LEFT_BARGE, getGlobalPositions().LEFT_BARGE_PATH)),
                new WaitCommand(0.1),
                new BargeScoreCommand(armevator, algaeManipulator, () -> driverController.getHID().getPOV() == 270).ignoringDisable(false)
            ).ignoringDisable(true)
        );

        buttonBoard.getButton(14).whileTrue(
            // new GoToArmevatorPoseState(armevator, L1_ARMEVATOR_POSITION).repeatedly()
            new InstantCommand(() -> armevator.goToPosNext(L1_ARMEVATOR_POSITION)).ignoringDisable(true)
        );

        buttonBoard.getButton(13).whileTrue(
            // new GoToArmevatorPoseState(armevator, L2_ARMEVATOR_POSITION).repeatedly()
            new InstantCommand(() -> armevator.goToPosNext(L2_ARMEVATOR_POSITION)).ignoringDisable(true)
        );

        buttonBoard.getButton(16 + 2).whileTrue(
            // new GoToArmevatorPoseState(armevator, L3_ARMEVATOR_POSITION).repeatedly()
            new InstantCommand(() -> armevator.goToPosNext(L3_ARMEVATOR_POSITION)).ignoringDisable(true)
        );

        buttonBoard.getButton(16 + 1).whileTrue(
            // new GoToArmevatorPoseState(armevator, L4_ARMEVATOR_POSITION).repeatedly()
            new InstantCommand(() -> armevator.goToPosNext(L4_ARMEVATOR_POSITION)).ignoringDisable(true)
        );

        buttonBoard.getButton(14).whileTrue(
            new GoToArmevatorPoseState(armevator, L1_ARMEVATOR_POSITION).repeatedly().beforeStarting(
                new WaitCommand(0.1)
            )
        );

        buttonBoard.getButton(13).whileTrue(
            new GoToArmevatorPoseState(armevator, L2_ARMEVATOR_POSITION).repeatedly().beforeStarting(
                new WaitCommand(0.1)
            )
        );

        buttonBoard.getButton(16 + 2).whileTrue(
            new GoToArmevatorPoseState(armevator, L3_ARMEVATOR_POSITION).repeatedly().beforeStarting(
                new WaitCommand(0.1)
            )
        );

        buttonBoard.getButton(16 + 1).whileTrue(
            new GoToArmevatorPoseState(armevator, L4_ARMEVATOR_POSITION).repeatedly().beforeStarting(
                new WaitCommand(0.1)
            )
        );

        //Reef buttons

        //Coral A/AB
        buttonBoard.getButton(11+16).onTrue(
            new ParallelCommandGroup(
                new InstantCommand(() -> drivetrain.setNextMiddlePath(getGlobalPositions().CORAL_AB, getGlobalPositions().SCORE_AB)).ignoringDisable(true),
                new InstantCommand(() -> drivetrain.setNextScorePose(getGlobalPositions().CORAL_AB, getGlobalPositions().CORAL_A)).ignoringDisable(true)
            )
        );

        //Coral B/AB
        buttonBoard.getButton(12+16).onTrue(
            new ParallelCommandGroup(
                new InstantCommand(() -> drivetrain.setNextMiddlePath(getGlobalPositions().CORAL_AB, getGlobalPositions().SCORE_AB)).ignoringDisable(true),
                new InstantCommand(() -> drivetrain.setNextScorePose(getGlobalPositions().CORAL_AB, getGlobalPositions().CORAL_B)).ignoringDisable(true)
            )        
        );

        //Coral C/CD
        buttonBoard.getButton(13+16).onTrue(
            new ParallelCommandGroup(
                new InstantCommand(() -> drivetrain.setNextMiddlePath(getGlobalPositions().CORAL_CD, getGlobalPositions().SCORE_CD)).ignoringDisable(true), 
                new InstantCommand(() -> drivetrain.setNextScorePose(getGlobalPositions().CORAL_CD, getGlobalPositions().CORAL_C)).ignoringDisable(true)
            )        
        );

        //Coral D/CD
        buttonBoard.getButton(15).onTrue(
            new ParallelCommandGroup(
                new InstantCommand(() -> drivetrain.setNextMiddlePath(getGlobalPositions().CORAL_CD, getGlobalPositions().SCORE_CD)).ignoringDisable(true), 
                new InstantCommand(() -> drivetrain.setNextScorePose(getGlobalPositions().CORAL_CD, getGlobalPositions().CORAL_D)).ignoringDisable(true)
            )
        );

        //Coral E/EF
        buttonBoard.getButton(14+16).onTrue(
            new ParallelCommandGroup(
                new InstantCommand(() -> drivetrain.setNextMiddlePath(getGlobalPositions().CORAL_EF, getGlobalPositions().SCORE_EF)).ignoringDisable(true), 
                new InstantCommand(() -> drivetrain.setNextScorePose(getGlobalPositions().CORAL_EF, getGlobalPositions().CORAL_E)).ignoringDisable(true)
            )
        );

        //Coral F/EF
        buttonBoard.getButton(12).onTrue(
            new ParallelCommandGroup(
                new InstantCommand(() -> drivetrain.setNextMiddlePath(getGlobalPositions().CORAL_EF, getGlobalPositions().SCORE_EF)).ignoringDisable(true), 
                new InstantCommand(() -> drivetrain.setNextScorePose(getGlobalPositions().CORAL_EF, getGlobalPositions().CORAL_F)).ignoringDisable(true)
            )
        );

        //Coral G/GH
        buttonBoard.getButton(5+16).onTrue(
            new ParallelCommandGroup(
                new InstantCommand(() -> drivetrain.setNextMiddlePath(getGlobalPositions().CORAL_GH, getGlobalPositions().SCORE_GH)).ignoringDisable(true), 
                new InstantCommand(() -> drivetrain.setNextScorePose(getGlobalPositions().CORAL_GH, getGlobalPositions().CORAL_G)).ignoringDisable(true)
            )
        );

        //Coral H/GH
        buttonBoard.getButton(6+16).onTrue(
            new ParallelCommandGroup(
                new InstantCommand(() -> drivetrain.setNextMiddlePath(getGlobalPositions().CORAL_GH, getGlobalPositions().SCORE_GH)).ignoringDisable(true), 
                new InstantCommand(() -> drivetrain.setNextScorePose(getGlobalPositions().CORAL_GH, getGlobalPositions().CORAL_H)).ignoringDisable(true)
            )
        );

        //Coral I/IJ
        buttonBoard.getButton(7+16).onTrue(
            new ParallelCommandGroup(
                new InstantCommand(() -> drivetrain.setNextMiddlePath(getGlobalPositions().CORAL_IJ, getGlobalPositions().SCORE_IJ)).ignoringDisable(true), 
                new InstantCommand(() -> drivetrain.setNextScorePose(getGlobalPositions().CORAL_IJ, getGlobalPositions().CORAL_I)).ignoringDisable(true)
            )        
        );

        //Coral J/IJ
        buttonBoard.getButton(8+16).onTrue(
            new ParallelCommandGroup(
                new InstantCommand(() -> drivetrain.setNextMiddlePath(getGlobalPositions().CORAL_IJ, getGlobalPositions().SCORE_IJ)).ignoringDisable(true), 
                new InstantCommand(() -> drivetrain.setNextScorePose(getGlobalPositions().CORAL_IJ, getGlobalPositions().CORAL_J)).ignoringDisable(true)
            )        );

        //Coral K/KL
        buttonBoard.getButton(9+16).onTrue(
            new ParallelCommandGroup(
                new InstantCommand(() -> drivetrain.setNextMiddlePath(getGlobalPositions().CORAL_KL, getGlobalPositions().SCORE_KL)).ignoringDisable(true), 
                new InstantCommand(() -> drivetrain.setNextScorePose(getGlobalPositions().CORAL_KL, getGlobalPositions().CORAL_K)).ignoringDisable(true)
            )        
        );

        //Coral L/KL
        buttonBoard.getButton(10+16).onTrue(
            new ParallelCommandGroup(
                new InstantCommand(() -> drivetrain.setNextMiddlePath(getGlobalPositions().CORAL_KL, getGlobalPositions().SCORE_KL)).ignoringDisable(true), 
                new InstantCommand(() -> drivetrain.setNextScorePose(getGlobalPositions().CORAL_KL, getGlobalPositions().CORAL_L)).ignoringDisable(true)
            )        
        );

        buttonBoard.getButton(1).onTrue(
            // new PathfindingState(drivetrain, getGlobalPositions().CORAL_STATION_LEFT)
            new InstantCommand(() -> drivetrain.setNextFeedPose(getGlobalPositions().CORAL_STATION_LEFT_CLOSE_POINT, getGlobalPositions().CORAL_STATION_LEFT_CLOSE)).ignoringDisable(true)
        );
        
        buttonBoard.getButton(3).whileTrue(
            new InstantCommand(() -> drivetrain.setNextFeedPose(getGlobalPositions().CORAL_STATION_LEFT_FAR_POINT, getGlobalPositions().CORAL_STATION_LEFT_FAR)).ignoringDisable(true)
        );

        buttonBoard.getButton(2).onTrue(
            // new PathfindingState(drivetrain, getGlobalPositions().CORAL_STATION_RIGHT)
            new InstantCommand(() -> drivetrain.setNextFeedPose(getGlobalPositions().CORAL_STATION_RIGHT_CLOSE_POINT, getGlobalPositions().CORAL_STATION_RIGHT_CLOSE)).ignoringDisable(true)
        );

        buttonBoard.getButton(4).onTrue(
            // new PathfindingState(drivetrain, getGlobalPositions().CORAL_STATION_RIGHT)
            new InstantCommand(() -> drivetrain.setNextFeedPose(getGlobalPositions().CORAL_STATION_RIGHT_FAR_POINT, getGlobalPositions().CORAL_STATION_RIGHT_FAR)).ignoringDisable(true)
        );
    }

    private void configureBackupBindings() {
        operatorController.leftTrigger().whileTrue(
            new FeederManipulatorCommand(feeder, coralManipulator, armevator)
        );

        operatorController.rightTrigger().whileTrue(
            new FeedState(feeder, -1.0).alongWith(new CoralOutakeState(coralManipulator, 1.0))
        );

        operatorController.a().whileTrue(
            new GoToArmevatorPoseState(armevator, L1_ARMEVATOR_POSITION).repeatedly()
        );

        operatorController.b().whileTrue(
            new GoToArmevatorPoseState(armevator, L2_ARMEVATOR_POSITION).repeatedly()
        );

        operatorController.x().whileTrue(
            new GoToArmevatorPoseState(armevator, L3_ARMEVATOR_POSITION).repeatedly()
        );

        operatorController.y().whileTrue(
            new GoToArmevatorPoseState(armevator, L4_ARMEVATOR_POSITION).repeatedly()
        );

        operatorController.rightBumper().whileTrue(
            new GoToArmevatorPoseState(armevator, ALGAE_ARMEVATOR_POSITION).alongWith(
                new AlgaeIntake(algaeManipulator)
            )
        );

        operatorController.leftBumper().whileTrue(
            new GoToArmevatorPoseState(armevator, ALGAE_ARMEVATOR_POSITION_TWO).alongWith(
                new AlgaeIntake(algaeManipulator)
            )
        );

        operatorController.povUp().whileTrue(
            new LowerState(climber)
        );

        operatorController.povDown().whileTrue(
            new ClimbState(climber)
        );

        operatorController.povLeft().whileTrue(
            new BargeScoreCommand(armevator, algaeManipulator, () -> driverController.povLeft().getAsBoolean())
        );
    }

    public static void registerNamedCommands() {
        // NamedCommands.registerCommand("Drop", new DropState(dropper).withTimeout(0.5)); //ex
        NamedCommands.registerCommand(
            "Score L4",
            new SequentialCommandGroup(
                new GoToArmevatorPoseState(armevator, L4_ARMEVATOR_POSITION)
                    .raceWith(new IdleState(coralManipulator, armevator::getArmRotation)), 
                new WaitCommand(0.2),
                new CoralOutakeState(coralManipulator, 1).withTimeout(.25)
            )
                
        );

        NamedCommands.registerCommand("Go to L4", 
            new GoToArmevatorPoseState(armevator, L4_ARMEVATOR_POSITION)
                .raceWith(new IdleState(coralManipulator, armevator::getArmRotation))
        );

        NamedCommands.registerCommand("Go home", new GoToArmevatorPoseState(armevator, HOME).withTimeout(0.1));

        NamedCommands.registerCommand("Feed to L4", 
            new SequentialCommandGroup(   
                new FeederManipulatorCommand(feeder, coralManipulator, armevator),
                new GoToArmevatorPosAndGrip(armevator, coralManipulator, L4_ARMEVATOR_POSITION)
            )
        );

        NamedCommands.registerCommand("Feed to L3", 
            new SequentialCommandGroup(   
                new FeederManipulatorCommand(feeder, coralManipulator, armevator),
                new GoToArmevatorPosAndGrip(armevator, coralManipulator, L3_ARMEVATOR_POSITION)
            )
        );

        NamedCommands.registerCommand("Next", 
            new GoToNextArmevatorPoseState(armevator)
                .raceWith(new IdleState(coralManipulator, armevator::getArmRotation))
        );

        NamedCommands.registerCommand("Feed", 
            new FeederManipulatorCommand(
                feeder, coralManipulator, armevator
            )
        );

        NamedCommands.registerCommand("Autonomous Feed", 
            new AutonomousFeedTillFirstLidar(
                feeder, coralManipulator, armevator, 1, 0.2
            )
        );

        NamedCommands.registerCommand("Feed to algae",
            new SequentialCommandGroup(   
                new FeederManipulatorCommand(feeder, coralManipulator, armevator),
                new GoToArmevatorPosAndGrip(armevator, coralManipulator, ALGAE_ARMEVATOR_POSITION),
                new AlgaeIntake(algaeManipulator)
            ).until(algaeManipulator::hasAlgae)
        );

        NamedCommands.registerCommand("Score L3", 
            new SequentialCommandGroup(
                new GoToArmevatorPosAndGrip(armevator, coralManipulator, L3_ARMEVATOR_POSITION),
                new WaitCommand(0.2),
                new CoralOutakeState(coralManipulator, 1).withTimeout(.25)
            )
        );
        
        NamedCommands.registerCommand("Auto Algee Low",
            new ParallelCommandGroup(
                new GoToArmevatorPoseState(armevator, ALGAE_ARMEVATOR_POSITION),
                new AlgaeIntake(algaeManipulator)
            ).until(algaeManipulator::hasAlgae)
        );

        NamedCommands.registerCommand("Auto Algee High",
            new ParallelCommandGroup(
                new GoToArmevatorPoseState(armevator, ALGAE_ARMEVATOR_POSITION_TWO),
                new AlgaeIntake(algaeManipulator)
            ).until(algaeManipulator::hasAlgae)
        );

        NamedCommands.registerCommand("Barge",
            new SequentialCommandGroup(
                new GoToArmevatorPoseState(armevator, BARGE_PREP_ARMEVATOR_POSITION),
                new GoToArmevatorPoseState(armevator, BARGE_ARMEVATOR_POSITION)
            ).raceWith(new AlgaeIntake(algaeManipulator)).withTimeout(5)   
        );

        NamedCommands.registerCommand("Barge Outake",
            new AlgaeOuttake(algaeManipulator).withTimeout(.5)
    );

        NamedCommands.registerCommand("Hold Algae Low",
            new ParallelCommandGroup(
                new GoToArmevatorPoseState(armevator, ALGAE_ARMEVATOR_POSITION),
                new AlgaeIntake(algaeManipulator)
            )        
        );
        
        NamedCommands.registerCommand("Hold Algae High",
        new ParallelCommandGroup(
            new GoToArmevatorPoseState(armevator, ALGAE_ARMEVATOR_POSITION_TWO),
            new AlgaeIntake(algaeManipulator)
        )        
    );
    }

    private void setupAutos() {
        autoChooser = new SendableChooser<>();

        SIDE_CHOOSER.setDefaultOption("Red", "red");
        SIDE_CHOOSER.addOption("Blue", "blue");
    
        autoTab = Shuffleboard.getTab("Auto");
        autoTab.add("AutoChooser", autoChooser);
        autoTab.add("SideChooser", SIDE_CHOOSER);

        // autoChooser.addOption("Wheel Diam", new PathPlannerAuto("Wheel Diam"));
        // autoChooser.addOption("Left Auto", new PathPlannerAuto("Left Two Piece auto", false));
        // autoChooser.addOption("Right Auto", new PathPlannerAuto("Right Two Piece auto", false));
        autoChooser.addOption("Right three piece auto", new PathPlannerAuto("Right three piece auto"));
        autoChooser.setDefaultOption("Left three piece auto", new PathPlannerAuto("Left three piece auto"));
        autoChooser.addOption("Middle Auto", new PathPlannerAuto("Short Auto", false));
        autoChooser.addOption("Left Four Piece", new PathPlannerAuto("Left four piece auto", false));
        autoChooser.addOption("right Four Piece", new PathPlannerAuto("Right four piece auto", false));
        // autoChooser.addOption("Right four piece minimal stops", new PathPlannerAuto("Right four piece minimal stops", false));
        // autoChooser.addOption("Left four piece minimal stops", new PathPlannerAuto("Left four piece minimal stops auto", false));
        autoChooser.addOption("One Coral Two Algee Left auto", new PathPlannerAuto("One Coral Two Algee Left auto", false));
        autoChooser.addOption("One Coral Two Algee Right auto", new PathPlannerAuto("One Coral Two Algee Right auto", false));
        autoChooser.addOption("Odometry test", new PathPlannerAuto("Wheel Diam"));
        // autoChooser.addOption("Barge Test", new PathPlannerAuto("Barge Test"));
        //autoChooser.setDefaultOption("Output name", new PathPlannerAuto("auto name", boolean mirror same field)); //ex
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
