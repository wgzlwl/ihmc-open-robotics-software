package us.ihmc.robotDataCommunication;

import us.ihmc.robotDataLogger.YoVariableServer;
import us.ihmc.robotDataLogger.logger.LogSettings;
import us.ihmc.util.PeriodicNonRealtimeThreadSchedulerFactory;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoInteger;

public class SingleThreadYoVariableConnectionTest
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final YoInteger sequence = new YoInteger("sequence", registry);
   
   
   public SingleThreadYoVariableConnectionTest()
   {
      // Add a bunch of variables
      for(int i = 0; i < 1000; i++)
      {
         new YoInteger("GarbageVariable" + i, registry);
      }
      
      
      YoVariableServer server = new YoVariableServer(getClass(), new PeriodicNonRealtimeThreadSchedulerFactory(), null, new LogSettings(false), 0.001);
      server.setMainRegistry(registry, null, null);
      
      server.start();
      while(true)
      {
         sequence.increment();
         server.update(System.nanoTime());

      }
   }
   
   public static void main(String[] args)
   {
      new SingleThreadYoVariableConnectionTest();
   }
   
}
