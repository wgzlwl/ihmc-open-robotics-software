package controller_msgs.msg.dds;

/**
 * Topic data type of the struct "ControllerCrashNotificationPacket" defined in "ControllerCrashNotificationPacket_.idl". Use this class to provide the TopicDataType to a Participant.
 *
 * This file was automatically generated from ControllerCrashNotificationPacket_.idl by us.ihmc.idl.generator.IDLGenerator.
 * Do not update this file directly, edit ControllerCrashNotificationPacket_.idl instead.
 */
public class ControllerCrashNotificationPacketPubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.ControllerCrashNotificationPacket>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::ControllerCrashNotificationPacket_";
   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   public ControllerCrashNotificationPacketPubSubType()
   {

   }

   public static int getMaxCdrSerializedSize()
   {
      return getMaxCdrSerializedSize(0);
   }

   public static int getMaxCdrSerializedSize(int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;

      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.ControllerCrashNotificationPacket data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.ControllerCrashNotificationPacket data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getStacktrace().length() + 1;

      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.ControllerCrashNotificationPacket data, us.ihmc.idl.CDR cdr)
   {

      cdr.write_type_9(data.getControllerCrashLocation());

      if (data.getStacktrace().length() <= 255)
         cdr.write_type_d(data.getStacktrace());
      else
         throw new RuntimeException("stacktrace field exceeds the maximum length");
   }

   public static void read(controller_msgs.msg.dds.ControllerCrashNotificationPacket data, us.ihmc.idl.CDR cdr)
   {

      data.setControllerCrashLocation(cdr.read_type_9());

      cdr.read_type_d(data.getStacktrace());
   }

   public static void staticCopy(controller_msgs.msg.dds.ControllerCrashNotificationPacket src, controller_msgs.msg.dds.ControllerCrashNotificationPacket dest)
   {
      dest.set(src);
   }

   @Override
   public void serialize(controller_msgs.msg.dds.ControllerCrashNotificationPacket data, us.ihmc.pubsub.common.SerializedPayload serializedPayload)
         throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.ControllerCrashNotificationPacket data)
         throws java.io.IOException
   {
      deserializeCDR.deserialize(serializedPayload);
      read(data, deserializeCDR);
      deserializeCDR.finishDeserialize();
   }

   @Override
   public final void serialize(controller_msgs.msg.dds.ControllerCrashNotificationPacket data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_9("controller_crash_location", data.getControllerCrashLocation());

      ser.write_type_d("stacktrace", data.getStacktrace());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.ControllerCrashNotificationPacket data)
   {
      data.setControllerCrashLocation(ser.read_type_9("controller_crash_location"));

      ser.read_type_d("stacktrace", data.getStacktrace());
   }

   @Override
   public controller_msgs.msg.dds.ControllerCrashNotificationPacket createData()
   {
      return new controller_msgs.msg.dds.ControllerCrashNotificationPacket();
   }

   @Override
   public int getTypeSize()
   {
      return us.ihmc.idl.CDR.getTypeSize(getMaxCdrSerializedSize());
   }

   @Override
   public java.lang.String getName()
   {
      return name;
   }

   public void serialize(controller_msgs.msg.dds.ControllerCrashNotificationPacket data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.ControllerCrashNotificationPacket data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }

   public void copy(controller_msgs.msg.dds.ControllerCrashNotificationPacket src, controller_msgs.msg.dds.ControllerCrashNotificationPacket dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public ControllerCrashNotificationPacketPubSubType newInstance()
   {
      return new ControllerCrashNotificationPacketPubSubType();
   }
}