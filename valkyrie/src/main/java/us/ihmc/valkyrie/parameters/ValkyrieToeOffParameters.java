package us.ihmc.valkyrie.parameters;

import us.ihmc.commonWalkingControlModules.configurations.ToeOffParameters;

public class ValkyrieToeOffParameters extends ToeOffParameters
{
   private final boolean runningOnRealRobot;

   public ValkyrieToeOffParameters(boolean runningOnRealRobot)
   {
      this.runningOnRealRobot = runningOnRealRobot;
   }

   @Override
   public boolean doToeOffIfPossible()
   {
      return true;
   }

   @Override
   public boolean doToeOffIfPossibleInSingleSupport()
   {
      return false;
   }

   @Override
   public boolean checkECMPLocationToTriggerToeOff()
   {
      // Used to be: target != RobotTarget.REAL_ROBOT;
      // Trying to see if that's really necessary (Sylvain)
      // It delays the toe-off to some extent which can cause some issues.
      return true;
   }

   @Override
   public double getECMPProximityForToeOff()
   {
      return runningOnRealRobot ? 0.0 : 0.03;
   }


   @Override
   public double getMinStepLengthForToeOff()
   {
      return ValkyriePhysicalProperties.footLength;
   }

   /**
    * To enable that feature, doToeOffIfPossible() return true is required.
    */
   @Override
   public boolean doToeOffWhenHittingAnkleLimit()
   {
      return false;
   }

   @Override
   public double getMaximumToeOffAngle()
   {
      return Math.toRadians(30.0);
   }

}
