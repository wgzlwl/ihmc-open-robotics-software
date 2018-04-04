package us.ihmc.quadrupedRobotics.controlModules;

import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyTaskspaceControlState;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.OrientationFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.MomentumRateCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualModelControlCommand;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PelvisOrientationTrajectoryCommand;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedForceControllerToolbox;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedBodyOrientationController;
import us.ihmc.quadrupedRobotics.estimator.GroundPlaneEstimator;
import us.ihmc.quadrupedRobotics.providers.QuadrupedPostureInputProviderInterface;
import us.ihmc.robotics.controllers.pidGains.GainCoupling;
import us.ihmc.robotics.controllers.pidGains.YoPID3DGains;
import us.ihmc.robotics.controllers.pidGains.implementations.DefaultPID3DGains;
import us.ihmc.robotics.controllers.pidGains.implementations.ParameterizedPID3DGains;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.OrientationFrame;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

import java.util.Collection;

public class QuadrupedBodyOrientationManager
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final QuadrupedBodyOrientationController.Setpoints setpoints = new QuadrupedBodyOrientationController.Setpoints();
   private final QuadrupedBodyOrientationController controller;
   private final YoPID3DGains gains;

   private final ParameterizedPID3DGains bodyOrientationGainsParameter;

   private final QuadrupedPostureInputProviderInterface postureProvider;
   private final GroundPlaneEstimator groundPlaneEstimator;

   private final FrameQuaternion bodyOrientationReference;
   private final OrientationFrame bodyOrientationReferenceFrame;

   private final RigidBodyTaskspaceControlState taskspaceControlState;

   private final QuadrupedForceControllerToolbox controllerToolbox;

   private final MomentumRateCommand angularMomentumCommand = new MomentumRateCommand();
   private final YoFrameVector bodyAngularWeight = new YoFrameVector("bodyAngularWeight", worldFrame, registry);
   private final FrameVector3D desiredAngularMomentumRate = new FrameVector3D();

   public QuadrupedBodyOrientationManager(QuadrupedForceControllerToolbox controllerToolbox, QuadrupedPostureInputProviderInterface postureProvider,
                                          YoVariableRegistry parentRegistry)
   {
      this.controllerToolbox = controllerToolbox;
      this.postureProvider = postureProvider;

      DefaultPID3DGains bodyOrientationDefaultGains = new DefaultPID3DGains();
      bodyOrientationDefaultGains.setProportionalGains(1000.0, 1000.0, 1000.0);
      bodyOrientationDefaultGains.setDerivativeGains(250.0, 250.0, 250.0);
      bodyOrientationDefaultGains.setIntegralGains(0.0, 0.0, 0.0, 0.0);
      bodyOrientationGainsParameter = new ParameterizedPID3DGains("_bodyOrientation", GainCoupling.NONE, false, bodyOrientationDefaultGains, registry);

      controller = new QuadrupedBodyOrientationController(controllerToolbox, registry);
      groundPlaneEstimator = controllerToolbox.getGroundPlaneEstimator();
      gains = controller.getGains();

      RigidBody body = controllerToolbox.getFullRobotModel().getBody();
      RigidBody elevator = controllerToolbox.getFullRobotModel().getElevator();
      ReferenceFrame baseFrame = controllerToolbox.getReferenceFrames().getCenterOFFourFeetZUpFrame();
      Collection<ReferenceFrame> trajectoryFrames = controllerToolbox.getTrajectoryFrames();

      taskspaceControlState = new RigidBodyTaskspaceControlState("Orientation", body, elevator, elevator, trajectoryFrames, body.getBodyFixedFrame(), baseFrame,
                                                                 controllerToolbox.getRuntimeEnvironment().getRobotTimestamp(), null,
                                                                 controllerToolbox.getRuntimeEnvironment().getGraphicsListRegistry(), registry);

      bodyOrientationReference = new FrameQuaternion();
      bodyOrientationReferenceFrame = new OrientationFrame(bodyOrientationReference);

      bodyAngularWeight.set(2.5, 2.5, 1.0);
      angularMomentumCommand.setAngularWeights(bodyAngularWeight);
      angularMomentumCommand.setSelectionMatrixForAngularControl();

      parentRegistry.addChild(registry);
   }

   public void initialize(FrameQuaternionReadOnly bodyOrientationEstimate)
   {
      setpoints.initialize(bodyOrientationEstimate);
      controller.reset();
   }

   private final FramePose3D tempPose = new FramePose3D();

   public boolean handleBodyOrientationTrajectoryCommands(PelvisOrientationTrajectoryCommand command, FrameQuaternion initialOrientation)
   {
      tempPose.setToNaN(initialOrientation.getReferenceFrame());
      tempPose.setOrientation(initialOrientation);
      return taskspaceControlState.handleOrientationTrajectoryCommand(command.getSO3Trajectory(), tempPose);
   }

   public void compute(FrameQuaternionReadOnly bodyOrientationDesired)
   {
      gains.set(bodyOrientationGainsParameter);

      bodyOrientationReference.setIncludingFrame(bodyOrientationDesired);
      bodyOrientationReference.changeFrame(bodyOrientationReferenceFrame.getParent());
      bodyOrientationReferenceFrame.setOrientationAndUpdate(bodyOrientationReference);

      setpoints.getBodyOrientation().changeFrame(bodyOrientationReferenceFrame);
      setpoints.getBodyOrientation().set(postureProvider.getBodyOrientationInput());
      setpoints.getBodyOrientation().changeFrame(worldFrame);
      double bodyOrientationYaw = setpoints.getBodyOrientation().getYaw();
      double bodyOrientationPitch = setpoints.getBodyOrientation().getPitch() + groundPlaneEstimator.getPitch(bodyOrientationYaw);
      double bodyOrientationRoll = setpoints.getBodyOrientation().getRoll();
      setpoints.getBodyOrientation().setYawPitchRoll(bodyOrientationYaw, bodyOrientationPitch, bodyOrientationRoll);

      setpoints.getBodyAngularVelocity().set(postureProvider.getBodyAngularRateInput());
      setpoints.getComTorqueFeedforward().setToZero();

      controller.compute(desiredAngularMomentumRate, setpoints, controllerToolbox.getTaskSpaceEstimates().getBodyAngularVelocity());

      desiredAngularMomentumRate.changeFrame(worldFrame);
      angularMomentumCommand.setAngularMomentumRate(desiredAngularMomentumRate);
      angularMomentumCommand.setAngularWeights(bodyAngularWeight);
   }

   public void getDesiredAngularMomentumRate(FrameVector3D angularMomentumRateToPack)
   {
      angularMomentumRateToPack.setIncludingFrame(desiredAngularMomentumRate);
   }

   public FeedbackControlCommand<?> createFeedbackControlTemplate()
   {
      return getFeedbackControlCommand();
   }

   public OrientationFeedbackControlCommand getFeedbackControlCommand()
   {
      return null;
   }

   public VirtualModelControlCommand<?> getVirtualModelControlCommand()
   {
      return angularMomentumCommand;
   }
}
