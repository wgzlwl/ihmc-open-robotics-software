package us.ihmc.llama;

import java.io.IOException;

import us.ihmc.llama.parameters.LlamaPositionBasedCrawlControllerParameters;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.robotModels.OutputWriter;
import us.ihmc.robotModels.FullQuadrupedRobotModel;
import us.ihmc.communication.net.NetClassList;
import us.ihmc.llama.simulation.LlamaGroundContactParameters;
import us.ihmc.llama.model.LlamaModelFactory;
import us.ihmc.llama.model.LlamaPhysicalProperties;
import us.ihmc.llama.model.LlamaSensorInformation;
import us.ihmc.llama.parameters.LlamaStateEstimatorParameters;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControlMode;
import us.ihmc.quadrupedRobotics.controller.QuadrupedSimulationFactory;
import us.ihmc.quadrupedRobotics.controller.position.states.QuadrupedPositionBasedCrawlControllerParameters;
import us.ihmc.quadrupedRobotics.estimator.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedRobotics.estimator.stateEstimator.QuadrupedSensorInformation;
import us.ihmc.quadrupedRobotics.model.QuadrupedModelFactory;
import us.ihmc.quadrupedRobotics.model.QuadrupedPhysicalProperties;
import us.ihmc.quadrupedRobotics.model.QuadrupedSimulationInitialPositionParameters;
import us.ihmc.robotics.dataStructures.parameter.ParameterRegistry;
import us.ihmc.quadrupedRobotics.simulation.GroundContactParameters;
import us.ihmc.quadrupedRobotics.simulation.QuadrupedGroundContactModelType;
import us.ihmc.quadrupedRobotics.simulation.QuadrupedParameterSet;
import us.ihmc.sensorProcessing.sensorProcessors.SensorTimestampHolder;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.simulationToolkit.outputWriters.PerfectSimulatedOutputWriter;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;

public class LlamaSimulationFactory
{
   private static final QuadrupedControlMode CONTROL_MODE = QuadrupedControlMode.FORCE;
   private final QuadrupedGroundContactModelType groundContactModelType = QuadrupedGroundContactModelType.FLAT;
   private static final double SIMULATION_DT = 0.00006;
   private static final double SIMULATION_GRAVITY = -9.81;
   private static final boolean USE_STATE_ESTIMATOR = false;
   private static final int RECORD_FREQUENCY = (int) (0.01 / SIMULATION_DT);
   private static final boolean SHOW_PLOTTER = true;
   private static final boolean USE_TRACK_AND_DOLLY = false;
   private static final boolean USE_NETWORKING = false;

   private QuadrupedSimulationFactory simulationFactory = new QuadrupedSimulationFactory();

   public SimulationConstructionSet createSimulation() throws IOException
   {
      QuadrupedModelFactory modelFactory = new LlamaModelFactory();
      QuadrupedPhysicalProperties physicalProperties = new LlamaPhysicalProperties();
      NetClassList netClassList = new LlamaNetClassList();
      SimulationConstructionSetParameters scsParameters = new SimulationConstructionSetParameters();
      QuadrupedSimulationInitialPositionParameters initialPositionParameters = new LlamaSimulationInitialPositionParameters();
      GroundContactParameters groundContactParameters = new LlamaGroundContactParameters();
      ParameterRegistry.getInstance().loadFromResources(QuadrupedParameterSet.SIMULATION_IDEAL.getPath());
      QuadrupedSensorInformation sensorInformation = new LlamaSensorInformation();
      StateEstimatorParameters stateEstimatorParameters = new LlamaStateEstimatorParameters();
      QuadrupedPositionBasedCrawlControllerParameters positionBasedCrawlControllerParameters = new LlamaPositionBasedCrawlControllerParameters();

      FullQuadrupedRobotModel fullRobotModel = modelFactory.createFullRobotModel();
      FloatingRootJointRobot sdfRobot = new FloatingRootJointRobot(modelFactory.createSdfRobot());

      SensorTimestampHolder timestampProvider = new LlamaTimestampProvider(sdfRobot);

      QuadrupedReferenceFrames referenceFrames = new QuadrupedReferenceFrames(fullRobotModel, physicalProperties);
      OutputWriter outputWriter = new PerfectSimulatedOutputWriter(sdfRobot, fullRobotModel);

      simulationFactory.setControlDT(SIMULATION_DT);
      simulationFactory.setSimulationDT(SIMULATION_DT);
      simulationFactory.setGravity(SIMULATION_GRAVITY);
      simulationFactory.setRecordFrequency(RECORD_FREQUENCY);
      simulationFactory.setGroundContactModelType(groundContactModelType);
      simulationFactory.setGroundContactParameters(groundContactParameters);
      simulationFactory.setModelFactory(modelFactory);
      simulationFactory.setSDFRobot(sdfRobot);
      simulationFactory.setSCSParameters(scsParameters);
      simulationFactory.setOutputWriter(outputWriter);
      simulationFactory.setShowPlotter(SHOW_PLOTTER);
      simulationFactory.setUseTrackAndDolly(USE_TRACK_AND_DOLLY);
      simulationFactory.setInitialPositionParameters(initialPositionParameters);
      simulationFactory.setFullRobotModel(fullRobotModel);
      simulationFactory.setPhysicalProperties(physicalProperties);
      simulationFactory.setControlMode(CONTROL_MODE);
      simulationFactory.setUseNetworking(USE_NETWORKING);
      simulationFactory.setTimestampHolder(timestampProvider);
      simulationFactory.setUseStateEstimator(USE_STATE_ESTIMATOR);
      simulationFactory.setStateEstimatorParameters(stateEstimatorParameters);
      simulationFactory.setSensorInformation(sensorInformation);
      simulationFactory.setReferenceFrames(referenceFrames);
      simulationFactory.setNetClassList(netClassList);
      simulationFactory.setPositionBasedCrawlControllerParameters(positionBasedCrawlControllerParameters);

      return simulationFactory.createSimulation();
   }

   public static void main(String[] commandLineArguments)
   {
      try
      {
         LlamaSimulationFactory simulationFactory = new LlamaSimulationFactory();
         SimulationConstructionSet scs = simulationFactory.createSimulation();
         scs.startOnAThread();
         scs.simulate();
      }
      catch (IOException ioException)
      {
         ioException.printStackTrace();
      }
   }
}
