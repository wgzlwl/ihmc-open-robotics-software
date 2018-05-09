package us.ihmc.quadrupedRobotics.controller;

import us.ihmc.robotics.stateMachine.extra.EventState;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;

public interface QuadrupedController extends EventState
{
   JointDesiredOutputListReadOnly getJointDesiredOutputList();
}
