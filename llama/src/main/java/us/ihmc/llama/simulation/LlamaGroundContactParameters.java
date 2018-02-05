package us.ihmc.llama.simulation;

import us.ihmc.quadrupedRobotics.simulation.GroundContactParameters;

public class LlamaGroundContactParameters implements GroundContactParameters
{
   @Override
   public double getZStiffness()
   {
      return 1000.0;
   }
   
   @Override
   public double getZDamping()
   {
      return 400.0;
   }
   
   @Override
   public double getXYStiffness()
   {
      return 4000.0;
   }
   
   @Override
   public double getXYDamping()
   {
      return 500.0;
   }
}
