package us.ihmc.robotics.trajectories.providers;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;

public class CurrentConfigurationProvider implements SE3ConfigurationProvider
{
   private final ReferenceFrame endEffectorFrame;

   public CurrentConfigurationProvider(ReferenceFrame endEffectorFrame)
   {
      this.endEffectorFrame = endEffectorFrame;
   }

   @Override
   public void getPosition(FramePoint3D positionToPack)
   {
      positionToPack.setToZero(endEffectorFrame);
   }

   @Override
   public void getOrientation(FrameQuaternion orientationToPack)
   {
      orientationToPack.setToZero(endEffectorFrame);
   }

   public void getPose(FramePose3D lastFootstepPose)
   {
      lastFootstepPose.setToZero(endEffectorFrame);
   }
}
