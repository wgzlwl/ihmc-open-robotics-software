package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import controller_msgs.msg.dds.BodyOrientationTrajectoryMessage;
import controller_msgs.msg.dds.PelvisOrientationTrajectoryMessage;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.humanoidRobotics.communication.controllerAPI.converter.FrameBasedCommand;
import us.ihmc.sensorProcessing.frames.ReferenceFrameHashCodeResolver;

import java.util.Random;

public class BodyOrientationTrajectoryCommand implements Command<BodyOrientationTrajectoryCommand, BodyOrientationTrajectoryMessage>,
      FrameBasedCommand<BodyOrientationTrajectoryMessage>, EpsilonComparable<BodyOrientationTrajectoryCommand>
{
   private boolean enableUserBodyControlDuringWalking = false;
   private final SO3TrajectoryControllerCommand so3Trajectory;

   public BodyOrientationTrajectoryCommand()
   {
      so3Trajectory = new SO3TrajectoryControllerCommand();
   }

   public BodyOrientationTrajectoryCommand(Random random)
   {
      so3Trajectory = new SO3TrajectoryControllerCommand(random);
   }

   @Override
   public void clear()
   {
      so3Trajectory.clear();
   }

   @Override
   public void set(BodyOrientationTrajectoryCommand other)
   {
      setEnableUserBodyControlDuringWalking(other.isEnableUserBodyControlDuringWalking());
      so3Trajectory.set(other.so3Trajectory);
   }

   @Override
   public void set(ReferenceFrameHashCodeResolver resolver, BodyOrientationTrajectoryMessage message)
   {
      setEnableUserBodyControlDuringWalking(message.getEnableUserBodyControlDuringWalking());
      so3Trajectory.set(resolver, message.getSo3Trajectory());
   }

   @Override
   public void set(BodyOrientationTrajectoryMessage message)
   {
      setEnableUserBodyControlDuringWalking(message.getEnableUserBodyControlDuringWalking());
      so3Trajectory.set(message.getSo3Trajectory());
   }

   /**
    * Allows setting this orientation {@link #SO3TrajectoryControllerCommand} trajectory command
    * from a pelvis pose {@link #SE3TrajectoryControllerCommand} trajectory command.
    */
   /*
   public void set(BodyTrajectoryCommand command)
   {
      setEnableUserBodyControlDuringWalking(command.isEnableUserPelvisControlDuringWalking());
      so3Trajectory.set(command.getSE3Trajectory());
   }
   */

   public boolean isEnableUserBodyControlDuringWalking()
   {
      return enableUserBodyControlDuringWalking;
   }

   public void setEnableUserBodyControlDuringWalking(boolean enableUserBodyControlDuringWalking)
   {
      this.enableUserBodyControlDuringWalking = enableUserBodyControlDuringWalking;
   }

   @Override
   public boolean epsilonEquals(BodyOrientationTrajectoryCommand other, double epsilon)
   {
      return so3Trajectory.epsilonEquals(other.so3Trajectory, epsilon);
   }

   public SO3TrajectoryControllerCommand getSO3Trajectory()
   {
      return so3Trajectory;
   }

   @Override
   public boolean isCommandValid()
   {
      return so3Trajectory.isCommandValid();
   }

   @Override
   public Class<BodyOrientationTrajectoryMessage> getMessageClass()
   {
      return BodyOrientationTrajectoryMessage.class;
   }

   @Override
   public boolean isDelayedExecutionSupported()
   {
      return true;
   }

   @Override
   public void setExecutionDelayTime(double delayTime)
   {
      so3Trajectory.setExecutionDelayTime(delayTime);
   }

   @Override
   public void setExecutionTime(double adjustedExecutionTime)
   {
      so3Trajectory.setExecutionTime(adjustedExecutionTime);
   }

   @Override
   public double getExecutionDelayTime()
   {
      return so3Trajectory.getExecutionDelayTime();
   }

   @Override
   public double getExecutionTime()
   {
      return so3Trajectory.getExecutionTime();
   }
}
