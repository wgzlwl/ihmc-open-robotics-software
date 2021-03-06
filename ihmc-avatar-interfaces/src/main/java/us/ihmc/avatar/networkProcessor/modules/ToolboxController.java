package us.ihmc.avatar.networkProcessor.modules;

import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.packets.Packet;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

public abstract class ToolboxController
{
   private static final boolean DEBUG = false;

   protected final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final StatusMessageOutputManager statusOutputManager;
   private final YoBoolean initialize = new YoBoolean("initialize" + registry.getName(), registry);

   public ToolboxController(StatusMessageOutputManager statusOutputManager, YoVariableRegistry parentRegistry)
   {
      this.statusOutputManager = statusOutputManager;
      parentRegistry.addChild(registry);
      requestInitialize();
   }

   public void requestInitialize()
   {
      initialize.set(true);
   }

   public void update()
   {
      if (initialize.getBooleanValue())
      {
         if (!initialize()) // Return until the initialization succeeds
            return;
         initialize.set(false);
      }

      try
      {
         updateInternal();
      }
      catch (Exception e)
      {
         if (DEBUG)
         {
            e.printStackTrace();
         }
      }
   }

   protected <T extends Packet<T>> void reportMessage(T statusMessage)
   {
      statusOutputManager.reportStatusMessage(statusMessage);
   }

   abstract protected void updateInternal() throws Exception;
   abstract protected boolean initialize();
   abstract protected boolean isDone();
}
