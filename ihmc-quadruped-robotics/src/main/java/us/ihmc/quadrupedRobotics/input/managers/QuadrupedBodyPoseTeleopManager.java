package us.ihmc.quadrupedRobotics.input.managers;

import com.google.common.util.concurrent.AtomicDouble;
import controller_msgs.msg.dds.BodyOrientationTrajectoryMessage;
import controller_msgs.msg.dds.SO3TrajectoryPointMessage;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.quadrupedRobotics.communication.packets.BodyOrientationPacket;
import us.ihmc.quadrupedRobotics.communication.packets.ComPositionPacket;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

import java.util.concurrent.atomic.AtomicReference;

public class QuadrupedBodyPoseTeleopManager
{
   private final PacketCommunicator packetCommunicator;

   private final AtomicDouble desiredCoMHeight = new AtomicDouble();
   private final AtomicDouble desiredOrientationYaw = new AtomicDouble();
   private final AtomicDouble desiredOrientationPitch = new AtomicDouble();
   private final AtomicDouble desiredOrientationRoll = new AtomicDouble();

   private final ComPositionPacket comPositionPacket = new ComPositionPacket();
//   private final BodyOrientationPacket bodyOrientationPacket = new BodyOrientationPacket();
   private final BodyOrientationTrajectoryMessage bodyOrientationMessage = new BodyOrientationTrajectoryMessage();
   private final SO3TrajectoryPointMessage orientationPoint = new SO3TrajectoryPointMessage();

   public QuadrupedBodyPoseTeleopManager(double initialCoMHeight, PacketCommunicator packetCommunicator)
   {
      this.packetCommunicator = packetCommunicator;
      desiredCoMHeight.set(initialCoMHeight);
   }

   public void setDesiredCoMHeight(double desiredCoMHeight)
   {
      this.desiredCoMHeight.set(desiredCoMHeight);
   }

   public void setDesiredBodyOrientation(double yaw, double pitch, double roll)
   {
      desiredOrientationYaw.set(yaw);
      desiredOrientationPitch.set(pitch);
      desiredOrientationRoll.set(roll);
   }

   public void update()
   {
      double comHeight = desiredCoMHeight.getAndSet(Double.NaN);
      double desiredYaw = desiredOrientationYaw.getAndSet(Double.NaN);
      double desiredPitch = desiredOrientationPitch.getAndSet(Double.NaN);
      double desiredRoll = desiredOrientationRoll.getAndSet(Double.NaN);

      if(!Double.isNaN(comHeight))
      {
         comPositionPacket.position.set(0.0, 0.0, comHeight);
         packetCommunicator.send(comPositionPacket);
      }

      if(!Double.isNaN(desiredYaw))
      {
         orientationPoint.getOrientation().setYawPitchRoll(desiredYaw, desiredPitch, desiredRoll);
         bodyOrientationMessage.getSo3Trajectory().getTaskspaceTrajectoryPoints().add().set(orientationPoint);
         packetCommunicator.send(bodyOrientationMessage);
//         bodyOrientationPacket.orientation.setYawPitchRoll(desiredYaw, desiredPitch, desiredRoll);
//         packetCommunicator.send(bodyOrientationPacket);
      }
   }
}
