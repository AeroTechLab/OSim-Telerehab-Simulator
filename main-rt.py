import sys
import math

from system_linearizer import SystemLinearizer

from pv_teleoperator import PVTeleoperator
from lqg_teleoperator import LQGTeleoperator
from wave_position_teleoperator import WaveTeleoperator
from lqg_prediction_teleoperator import LQGPredTeleoperator
from lqg_feedback_teleoperator import LQGFFBTeleoperator
from lqg_feedback_prediction_teleoperator import LQGFFBPredTeleoperator

from mtdpc_stabilizer import MTDPCStabilizer

from simple_plant import SimplePlant
import opensim

from numpy import random, ravel, max, average
from scipy.signal import butter, lfilter, freqz
from matplotlib import pyplot

SIM_TIME_STEPS_NUMBER = 2000

ENVIRONMENT_IDS = [ "fm", "r", "pa", "ca", "c"  ]
ENVIRONMENT_NAMES = [ "Free Motion", "Resistive", "Power Assistive", "Coordination Assistive", "Competitive"  ]
CONTROLLER_IDS = [ "pv", "wv", "lqg", "lqgp", "lqgf", "lqgfp" ]
CONTROLLER_NAMES = [ "PV", "Wave Variables", "LQG", "LQG Prediction", "LQG-FFB", "LQG-FFB Prediction" ]

MASTER_KP = 4.0
SLAVE_KP = 8.0

MASTER_IMPEDANCE = ( 1.0, 4.0, 0.0 )
SLAVE_IMPEDANCE = ( 1.0, 2.0, 0.0 )
STABILIZER_FACTOR = 0.5

NET_TIME_STEP = 0.02

useDelay = False
useVariableDelay = False
useDynamicImpedance = False
useStabilizer = False
environmentType = ENVIRONMENT_IDS.index( sys.argv[ 1 ] )
controllerType = CONTROLLER_IDS.index( sys.argv[ 2 ] )
for arg in sys.argv[ 3: ]:
  if arg == '--delay': useDelay = True
  elif arg == '--jitter': useVariableDelay = True
  elif arg == '--dynamic': useDynamicImpedance = True
  elif arg == '--stabilizer': useStabilizer = True
print( environmentType, controllerType, useDelay, useVariableDelay, useDynamicImpedance, useStabilizer )

NET_DELAY_AVG = 0.2 if useDelay else 0.0
NET_DELAY_VAR = 0.1 if ( useDelay and useVariableDelay ) else 0.0

Teleoperator = PVTeleoperator
if controllerType == 1: Teleoperator = WaveTeleoperator
elif controllerType == 2: Teleoperator = LQGTeleoperator
elif controllerType == 3: Teleoperator = LQGPredTeleoperator
elif controllerType == 4: Teleoperator = LQGFFBTeleoperator
elif controllerType == 5: Teleoperator = LQGFFBPredTeleoperator

masterLinearizer = SystemLinearizer()
slaveLinearizer = SystemLinearizer()
masterTeleoperator = Teleoperator( MASTER_IMPEDANCE, NET_TIME_STEP )
slaveTeleoperator = Teleoperator( SLAVE_IMPEDANCE, NET_TIME_STEP )
masterStabilizer = MTDPCStabilizer( STABILIZER_FACTOR, NET_TIME_STEP )
slaveStabilizer = MTDPCStabilizer( STABILIZER_FACTOR, NET_TIME_STEP )

masterToSlaveQueue = [ ( ( 0.0, 0.0, 0.0 ), 0.0 ) ]
masterToSlaveTimesQueue = [ 0.0 ]
masterToSlaveDelays = [ 0.0 ]
slaveToMasterQueue = [ ( ( 0.0, 0.0, 0.0 ), 0.0 ) ]
slaveToMasterTimesQueue = [ 0.0 ]
slaveToMasterDelays = [ 0.0 ]

random.seed( 0 )
setpoints = ravel( 2 * ( random.rand( 1, SIM_TIME_STEPS_NUMBER ) - 0.5 ) ).tolist()
b, a = butter( 2, 0.01, btype='low', analog=False )
setpoints = lfilter( b, a, setpoints )

try:
  model = opensim.Model()
  
  model.setUseVisualizer( True )
  
  model.setName( 'TelerehabSimulator' )
  model.setGravity( opensim.Vec3( 0, 0, 0 ) )

  master = opensim.Body( 'master', MASTER_IMPEDANCE[ 0 ], opensim.Vec3( 0, 0, 0 ), opensim.Inertia( 0, 0, 0 ) )
  model.addBody( master )
  slave = opensim.Body( 'slave', SLAVE_IMPEDANCE[ 0 ], opensim.Vec3( 0, 0, 0 ), opensim.Inertia( 0, 0, 0 ) )
  model.addBody( slave )

  ground = model.getGround()
  masterToGround = opensim.SliderJoint( 'master2ground', ground, master )
  model.addJoint( masterToGround )
  slaveToGround = opensim.SliderJoint( 'slave2ground', ground, slave )
  model.addJoint( slaveToGround )
  
  blockMesh = opensim.Brick( opensim.Vec3( 0.5, 0.5, 0.5 ) )
  blockMesh.setColor( opensim.Red )
  masterOffsetFrame = opensim.PhysicalOffsetFrame()
  masterOffsetFrame.setParentFrame( master )
  masterOffsetFrame.setOffsetTransform( opensim.Transform( opensim.Vec3( 0, 0, 0.5 ) ) )
  master.addComponent( masterOffsetFrame )
  masterOffsetFrame.attachGeometry( blockMesh.clone() )
  blockMesh.setColor( opensim.Blue )
  slaveOffsetFrame = opensim.PhysicalOffsetFrame()
  slaveOffsetFrame.setParentFrame( slave )
  slaveOffsetFrame.setOffsetTransform( opensim.Transform( opensim.Vec3( 0, 0, -0.5 ) ) )
  slave.addComponent( slaveOffsetFrame )
  slaveOffsetFrame.attachGeometry( blockMesh.clone() )
  
  masterCoordinate = masterToGround.updCoordinate()
  masterInputActuator = opensim.CoordinateActuator( 'masterInput' )
  masterInputActuator.setCoordinate( masterCoordinate )
  model.addForce( masterInputActuator )
  masterFeedbackActuator = opensim.CoordinateActuator( 'masterFeedback' )
  masterFeedbackActuator.setCoordinate( masterCoordinate )
  model.addForce( masterFeedbackActuator )
  
  slaveCoordinate = slaveToGround.updCoordinate()
  slaveInputActuator = opensim.CoordinateActuator( 'slaveInput' )
  slaveInputActuator.setCoordinate( slaveCoordinate )
  model.addForce( slaveInputActuator )
  slaveFeedbackActuator = opensim.CoordinateActuator( 'slaveFeedback' )
  slaveFeedbackActuator.setCoordinate( slaveCoordinate )
  model.addForce( slaveFeedbackActuator )
  
  #model.finalizeFromProperties()
  #model.printBasicInfo()
  
  systemState = model.initSystem()

  masterInputActuator.overrideActuation( systemState, True )
  masterFeedbackActuator.overrideActuation( systemState, True )
  slaveInputActuator.overrideActuation( systemState, True )
  slaveFeedbackActuator.overrideActuation( systemState, True )
  
  masterCoordinate.setValue( systemState, 0.0 )
  slaveCoordinate.setValue( systemState, 0.0 )

  manager = opensim.Manager( model )
  systemState.setTime( 0 )
  manager.initialize( systemState )
  
  referencePlant = SimplePlant( MASTER_IMPEDANCE[ 0 ], MASTER_IMPEDANCE[ 1 ], MASTER_IMPEDANCE[ 2 ], NET_TIME_STEP )
  referenceOutput = ( 0.0, 0.0, 0.0 )
  
  inputSilo = model.updVisualizer().updInputSilo()
  visualizer = model.updVisualizer().updSimbodyVisualizer()
  
  #viz = model.updVisualizer().updSimbodyVisualizer()
  #viz.setCameraFieldOfView( opensim.SimTK_PI / 3 )
  
  #viz.setCameraTransform( opensim.Transform( opensim.Vec3( 0, 0, 0.5 ) ) )
  #viz.setBackgroundType( viz.SolidColor )
  #viz.setBackgroundColor( opensim.White )
  
  referencePositions = [ 0.0 ]
  positionErrorRMS = 0.0
  inertiaErrorRMS = 0.0
  dampingErrorRMS = 0.0
  stiffnessErrorRMS = 0.0
  timeSteps = [ 0.0 ]
  masterPositions = [ 0.0 ]
  slavePositions = [ 0.0 ]
  masterDelayedPositions = [ 0.0 ]
  slaveDelayedPositions = [ 0.0 ]
  masterSetpointPositions = [ 0.0 ]
  slaveSetpointPositions = [ 0.0 ]
  masterInputs = [ 0.0 ]
  slaveInputs = [ 0.0 ]
  masterFeedbackInputs = [ 0.0 ]
  slaveFeedbackInputs = [ 0.0 ]
  masterInputInertias = [ 0.0 ]
  slaveInertias = [ 0.0 ]
  masterInputDampings = [ 0.0 ]
  slaveDampings = [ 0.0 ]
  masterInputStiffnesses = [ 0.0 ]
  slaveStiffnesses = [ 0.0 ]
  masterInputEnergy = [ 0.0 ]
  slaveFeedbackEnergy = [ 0.0 ]
  masterNetEnergy = [ 0.0 ]
  masterDissipatedEnergy = [ 0.0 ]
  referenceEnergy = [ 0.0 ]
  
  isRunning = True
  timeStepIndex = 0
  while isRunning:
    
    masterInput = 0.0
    slaveInput = 0.0    
    while inputSilo.isAnyUserInput():
      key = inputSilo.takeKeyHitKeyOnly()
      if key == opensim.SimTKVisualizerInputListener.KeyEsc: isRunning = False
      elif key == opensim.SimTKVisualizerInputListener.KeyUpArrow: slaveInput = -1.0
      elif key == opensim.SimTKVisualizerInputListener.KeyDownArrow: slaveInput = 1.0
      elif key == opensim.SimTKVisualizerInputListener.KeyLeftArrow: masterInput = -1.0
      elif key == opensim.SimTKVisualizerInputListener.KeyRightArrow: masterInput = 1.0
    masterInput = 0.2 * masterInput + 0.8 * masterInputs[ -1 ]
    slaveInput = 0.2 * slaveInput + 0.8 * slaveInputs[ -1 ]
    
    simTime = timeStepIndex * NET_TIME_STEP
    
    # master dynamics
    masterOutput = ( masterCoordinate.getValue( systemState ),
                     masterCoordinate.getSpeedValue( systemState ),
                     masterCoordinate.getAccelerationValue( systemState ) )
    masterDamping = MASTER_IMPEDANCE[ 1 ] * masterOutput[ 1 ]
    masterInputActuator.setOverrideActuation( systemState, masterInput - masterDamping )
    # receive master delayed setpoints
    while simTime >= slaveToMasterTimesQueue[ 0 ]:
      slaveToMasterTimesQueue.pop( 0 )
      slaveDelayedOutput, slaveDelayedInput = slaveToMasterQueue.pop( 0 )
      if len( slaveToMasterQueue ) == 0: break
    # linearize master system
    masterLinearizer.AddSample( masterOutput[ 0 ], masterOutput[ 1 ], masterOutput[ 2 ], masterInput, slaveFeedbackInputs[ -1 ] )
    masterInputImpedance, masterOutputImpedance, masterPlantImpedance = masterLinearizer.IdentifySystem( MASTER_IMPEDANCE )
    if useDynamicImpedance: masterTeleoperator.SetSystem( masterPlantImpedance )
    # master control
    controlOutput = masterTeleoperator.Process( masterOutput, slaveDelayedOutput, masterInput, slaveDelayedInput, slaveToMasterDelays[ -1 ] )
    slaveFeedbackInput, slaveCorrectedOutput, masterState = controlOutput
    if useStabilizer: slaveFeedbackInput = masterStabilizer.Process( slaveFeedbackInput, MASTER_KV, masterOutput[ 1 ] )
    masterFeedbackActuator.setOverrideActuation( systemState, slaveFeedbackInput )
    #masterOutput = masterPlant.Process( masterInput + slaveFeedbackInput )
    # send slave delayed setpoints
    masterToSlaveQueue.append( ( masterState, masterInput ) )
    masterToSlaveDelays.append( NET_DELAY_AVG + NET_DELAY_VAR * random.randint( 0, 1000 ) / 1000.0 )
    masterToSlaveTimesQueue.append( simTime + masterToSlaveDelays[ -1 ] )

    # slave dynamics
    slaveOutput = ( slaveCoordinate.getValue( systemState ),
                    slaveCoordinate.getSpeedValue( systemState ),
                    slaveCoordinate.getAccelerationValue( systemState ) )
    slaveDamping = SLAVE_IMPEDANCE[ 1 ] * slaveOutput[ 1 ]
    slaveInputActuator.setOverrideActuation( systemState, slaveInput - slaveDamping )
    # receive slave delayed setpoints
    while simTime >= masterToSlaveTimesQueue[ 0 ]:
      masterToSlaveTimesQueue.pop( 0 )
      masterDelayedOutput, masterDelayedInput = masterToSlaveQueue.pop( 0 )
      if len( masterToSlaveQueue ) == 0: break
    # linearize slave system
    slaveLinearizer.AddSample( slaveOutput[ 0 ], slaveOutput[ 1 ], slaveOutput[ 2 ], slaveInput, masterFeedbackInputs[ -1 ] )
    slaveInputImpedance, slaveOutputImpedance, slavePlantImpedance = slaveLinearizer.IdentifySystem( SLAVE_IMPEDANCE )
    if useDynamicImpedance: slaveTeleoperator.SetSystem( slavePlantImpedance )
    # slave control
    controlOutput = slaveTeleoperator.Process( slaveOutput, masterDelayedOutput, slaveInput, masterDelayedInput, masterToSlaveDelays[ -1 ] )
    masterFeedbackInput, masterCorrectedOutput, slaveState = controlOutput
    #if useStabilizer: masterFeedbackInput = slaveStabilizer.Process( masterFeedbackInput, SLAVE_KV, slaveOutput[ 1 ] )
    slaveFeedbackActuator.setOverrideActuation( systemState, masterFeedbackInput )
    #slaveOutput = slavePlant.Process( slaveInput + masterFeedbackInput )
    # send master delayed setpoints
    slaveToMasterQueue.append( ( slaveState, slaveInput ) )
    slaveToMasterDelays.append( NET_DELAY_AVG + NET_DELAY_VAR * random.randint( 0, 1000 ) / 1000.0 )
    slaveToMasterTimesQueue.append( simTime + slaveToMasterDelays[ -1 ] )
    
    # system update
    systemState = manager.integrate( simTime )
    
    # reference update 
    referenceOutput = referencePlant.Process( masterInput - masterDamping + slaveInput - slaveDamping )
    
    # perfomance calculation
    positionErrorRMS += ( masterOutput[ 0 ] - slaveOutput[ 0 ] )**2 / SIM_TIME_STEPS_NUMBER
    inertiaErrorRMS += ( masterInputImpedance[ 0 ] - slaveOutputImpedance[ 0 ] )**2 / SIM_TIME_STEPS_NUMBER
    dampingErrorRMS += ( masterInputImpedance[ 1 ] - slaveOutputImpedance[ 1 ] )**2 / SIM_TIME_STEPS_NUMBER
    stiffnessErrorRMS += ( masterInputImpedance[ 2 ] - slaveOutputImpedance[ 2 ] )**2 / SIM_TIME_STEPS_NUMBER
    
    # data logging
    timeSteps.append( simTime )
    referencePositions.append( referenceOutput[ 0 ] )
    masterPositions.append( masterOutput[ 0 ] )
    slavePositions.append( slaveOutput[ 0 ] )
    masterDelayedPositions.append( masterDelayedOutput[ 0 ] )
    slaveDelayedPositions.append( slaveDelayedOutput[ 0 ] )
    masterSetpointPositions.append( slaveCorrectedOutput[ 0 ] )
    slaveSetpointPositions.append( masterCorrectedOutput[ 0 ] )
    masterInputs.append( masterInput )
    slaveInputs.append( slaveInput )
    masterFeedbackInputs.append( masterFeedbackInput )
    slaveFeedbackInputs.append( slaveFeedbackInput )
    masterInputInertias.append( masterInputImpedance[ 0 ] )
    slaveInertias.append( slaveOutputImpedance[ 0 ] )
    masterInputDampings.append( masterInputImpedance[ 1 ] )
    slaveDampings.append( slaveOutputImpedance[ 1 ] )
    masterInputStiffnesses.append( masterInputImpedance[ 2 ] )
    slaveStiffnesses.append( slaveOutputImpedance[ 2 ] )
    masterInputPower = masterInput * masterOutput[ 1 ]
    masterInputEnergy.append( masterInputEnergy[ -1 ] + masterInputPower * NET_TIME_STEP )
    slaveFeedbackPower = slaveFeedbackInput * masterOutput[ 1 ]
    slaveFeedbackEnergy.append( slaveFeedbackEnergy[ -1 ] + slaveFeedbackPower * NET_TIME_STEP )
    masterNetEnergy.append( masterNetEnergy[ -1 ] + ( masterInputPower + slaveFeedbackPower ) * NET_TIME_STEP )
    masterDissipatedPower = MASTER_KV * masterOutput[ 1 ] * masterOutput[ 1 ]
    if masterDissipatedPower < 0.0: masterDissipatedPower = 0.0
    masterDissipatedEnergy.append( masterDissipatedEnergy[ -1 ] + masterDissipatedPower * NET_TIME_STEP )
    referencePower = ( masterInput + slaveInput ) * referenceOutput[ 1 ]
    referenceEnergy.append( referenceEnergy[ -1 ] + referencePower * NET_TIME_STEP )
    
    timeStepIndex += 1
    
  #model.setUseVisualizer( False )
  
  positionErrorRMS = math.sqrt( positionErrorRMS )
  inertiaErrorRMS = math.sqrt( inertiaErrorRMS )
  dampingErrorRMS = math.sqrt( dampingErrorRMS )
  stiffnessErrorRMS = math.sqrt( stiffnessErrorRMS )
  print( "{:.3f} {:.3f} {:.3f} {:.3f} {:.3f}".format( positionErrorRMS, inertiaErrorRMS, dampingErrorRMS, stiffnessErrorRMS, masterNetEnergy[ -1 ] ) )
  
  pyplot.subplot( 311, xlim=[ 0.0, SIM_TIME_STEPS_NUMBER * NET_TIME_STEP ], ylim=[ -0.2, 0.2 ] )
  pyplot.title( 'Teleoperation w/ {} Environment (delay={}±{}[s])\n{} Controller ( position RMS error={:.3f}, impedance RMS error=({:.3f},{:.3f},{:.3f}) )\n'.format( 
                ENVIRONMENT_NAMES[ environmentType ], NET_DELAY_AVG, NET_DELAY_VAR, CONTROLLER_NAMES[ controllerType ], positionErrorRMS, inertiaErrorRMS, dampingErrorRMS, stiffnessErrorRMS ), fontsize=15 )
  pyplot.ylabel( 'Position [m]', fontsize=10 )
  pyplot.tick_params( axis='x', labelsize=0 )
  pyplot.plot( timeSteps, referencePositions, 'k:' )
  #pyplot.plot( timeSteps, masterDelayedPositions, 'b:', timeSteps, slaveDelayedPositions, 'r:' )
  #pyplot.plot( timeSteps, masterSetpointPositions, 'b--', timeSteps, slaveSetpointPositions, 'r--' )
  pyplot.plot( timeSteps, masterPositions, 'b-', timeSteps, slavePositions, 'r-' )
  pyplot.legend( [ 'reference', 'master', 'slave' ] )
  #pyplot.legend( [ 'reference', 'master-delayed', 'slave-delayed', 'master-setpoint', 'slave-setpoint', 'master', 'slave' ] )
  #pyplot.legend( [ 'master-delayed', 'slave-delayed', 'master-setpoint', 'slave-setpoint', 'master', 'slave' ] )
  
  #pyplot.subplot( 312, xlim=[ 0.0, SIM_TIME_STEPS_NUMBER * NET_TIME_STEP ], ylim=[ -1.5, 1.5 ] )
  pyplot.subplot( 312, xlim=[ 0.0, SIM_TIME_STEPS_NUMBER * NET_TIME_STEP ], ylim=[ -0.2, 0.2 ] )
  #pyplot.ylabel( 'Force [N]', fontsize=10 )
  #pyplot.tick_params( axis='x', labelsize=0 )
  #pyplot.plot( timeSteps, masterInputs, 'g-', timeSteps, slaveInputs, 'm-' )
  #pyplot.plot( timeSteps, masterFeedbackInputs, 'b-', timeSteps, slaveFeedbackInputs, 'r-' )
  #pyplot.legend( [ 'master-input', 'slave-input', 'master-feedback', 'slave-feedback' ] )
  #pyplot.subplot( 313, xlim=[ 0.0, SIM_TIME_STEPS_NUMBER * NET_TIME_STEP ], ylim=[ -0.75, 0.75 ] )
  pyplot.ylabel( 'Energy [J]', fontsize=10 )
  #pyplot.xlabel( 'Time [s]', fontsize=10 )
  pyplot.plot( timeSteps, masterInputEnergy, 'b-', timeSteps, slaveFeedbackEnergy, 'r-', timeSteps, masterNetEnergy, 'g-', timeSteps, masterDissipatedEnergy, 'm-' )
  pyplot.plot( timeSteps, referenceEnergy, 'k:' )
  pyplot.legend( [ 'master-input', 'slave-feeback', 'net-work', 'master-dissipated' ] )
  
  pyplot.subplot( 313, xlim=[ 0.0, SIM_TIME_STEPS_NUMBER * NET_TIME_STEP ], ylim=[ -2.5, 8.5 ] )
  pyplot.ylabel( 'Impedance', fontsize=10 )
  pyplot.xlabel( 'Time [s]', fontsize=10 )
  pyplot.plot( timeSteps, masterInputInertias, 'b--', timeSteps, slaveInertias, 'r--' )
  pyplot.plot( timeSteps, masterInputDampings, 'b-.', timeSteps, slaveDampings, 'r-.' )
  pyplot.plot( timeSteps, masterInputStiffnesses, 'b:', timeSteps, slaveStiffnesses, 'r:' )
  pyplot.legend( [ 'master-inertia', 'slave-inertia', 'master-damping', 'slave-damping', 'master-stiffness', 'slave-stiffness' ] )
  pyplot.show()
except Exception as e:
  print( e )
