package us.ihmc.atlas.joystickBasedStepping;

import controller_msgs.msg.dds.BDIBehaviorCommandPacket;
import javafx.application.Application;
import javafx.application.Platform;
import javafx.stage.Stage;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.joystickBasedJavaFXController.JoystickBasedSteppingMainUI;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.commons.PrintTools;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.ros2.Ros2Node;

public class AtlasJoystickBasedSteppingApplication extends Application
{
   private JoystickBasedSteppingMainUI ui;
   private final Ros2Node ros2Node = ROS2Tools.createRos2Node(PubSubImplementation.FAST_RTPS, "ihmc_atlas_xbox_joystick_control");
   private IHMCROS2Publisher<BDIBehaviorCommandPacket> bdiBehaviorcommandPublisher;

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      String robotTargetString = getParameters().getNamed().getOrDefault("robotTarget", "REAL_ROBOT");
      RobotTarget robotTarget = RobotTarget.valueOf(robotTargetString);
      PrintTools.info("-------------------------------------------------------------------");
      PrintTools.info("  -------- Loading parameters for RobotTarget: " + robotTarget + "  -------");
      PrintTools.info("-------------------------------------------------------------------");
      AtlasRobotModel atlasRobotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ, robotTarget, false);
      String robotName = atlasRobotModel.getSimpleRobotName();
      bdiBehaviorcommandPublisher = ROS2Tools.createPublisher(ros2Node, BDIBehaviorCommandPacket.class,
                                                              ControllerAPIDefinition.getSubscriberTopicNameGenerator(robotName));
      AtlasKickAndPunchMessenger atlasKickAndPunchMessenger = new AtlasKickAndPunchMessenger(ros2Node, robotName);

      ui = new JoystickBasedSteppingMainUI(robotName, primaryStage, ros2Node, atlasRobotModel, atlasRobotModel.getWalkingControllerParameters(),
                                           atlasKickAndPunchMessenger, atlasKickAndPunchMessenger, atlasKickAndPunchMessenger);
   }

   @Override
   public void stop() throws Exception
   {
      super.stop();
      ui.stop();

      if (bdiBehaviorcommandPublisher != null)
         bdiBehaviorcommandPublisher.publish(HumanoidMessageTools.createBDIBehaviorCommandPacket(true));

      Platform.exit();
   }

   /**
    * 
    * @param args should either be {@code --robotTarget=SCS} or {@code --robotTarget=REAL_ROBOT}.
    */
   public static void main(String[] args)
   {
      launch(args);
   }
}
