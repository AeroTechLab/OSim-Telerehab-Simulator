import math

from system_linearizer import SystemLinearizer

from pv_teleoperator import PVTeleoperator
from lqg_teleoperator import LQGTeleoperator
from wave_position_teleoperator import WaveTeleoperator
from lqg_prediction_teleoperator import LQGPredTeleoperator

import opensim

from numpy import random, ravel, max, average
from scipy.signal import butter, lfilter, freqz
from matplotlib import pyplot

SIM_TIME_STEPS_NUMBER = 1000#5000

MASTER_KP = 10.0
MASTER_KV = 1.0
SLAVE_KP = 5.0
SLAVE_KV = 0.5

USE_DELAY = False
USE_DYNAMIC_IMPEDANCE = True#False
ENVIRONMENT_NAMES = [ "Free Motion", "Resistive Spring-Damper", "Assistive", "Resistive Opposite"  ]
ENVIRONMENT_TYPE = 0
CONTROLLER_NAMES = [ "PV", "LQG", "Wave Variables", "LQG Prediction"  ]
CONTROLLER_TYPE = 2

NET_TIME_STEP = 0.02
NET_DELAY_AVG = 0.2 if USE_DELAY else 0.0
NET_DELAY_VAR = 0.1 if USE_DELAY else 0.0

Teleoperator = PVTeleoperator
if CONTROLLER_TYPE == 1: Teleoperator = LQGTeleoperator
elif CONTROLLER_TYPE == 2: Teleoperator = WaveTeleoperator
elif CONTROLLER_TYPE == 3: Teleoperator = LQGPredTeleoperator

OPERATOR_IMPEDANCE = ( 1.0, 0.0, 0.0 )
masterLinearizer = SystemLinearizer()
slaveLinearizer = SystemLinearizer()
masterTeleoperator = Teleoperator( OPERATOR_IMPEDANCE, NET_TIME_STEP )
slaveTeleoperator = Teleoperator( OPERATOR_IMPEDANCE, NET_TIME_STEP )

masterToSlaveQueue = [ ( ( 0.0, 0.0, 0.0 ), 0.0, OPERATOR_IMPEDANCE ) ]
masterToSlaveTimesQueue = [ 0.0 ]
masterToSlaveDelays = [ 0.0 ]
slaveToMasterQueue = [ ( ( 0.0, 0.0, 0.0 ), 0.0, OPERATOR_IMPEDANCE ) ]
slaveToMasterTimesQueue = [ 0.0 ]
slaveToMasterDelays = [ 0.0 ]

random.seed( 0 )
setpoints = ravel( 2 * ( random.rand( 1, SIM_TIME_STEPS_NUMBER ) - 0.5 ) ).tolist()
b, a = butter( 2, 0.01, btype='low', analog=False )
setpoints = lfilter( b, a, setpoints )

try:
  model = opensim.Model()
  
  #model.setUseVisualizer( True )
  
  model.setName( 'TelerehabSimulator' )
  model.setGravity( opensim.Vec3( 0, 0, 0 ) )

  master = opensim.Body( 'master', OPERATOR_IMPEDANCE[ 0 ], opensim.Vec3( 0, 0, 0 ), opensim.Inertia( 0, 0, 0 ) )
  model.addBody( master )
  slave = opensim.Body( 'slave', OPERATOR_IMPEDANCE[ 0 ], opensim.Vec3( 0, 0, 0 ), opensim.Inertia( 0, 0, 0 ) )
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

  #viz = model.updVisualizer().updSimbodyVisualizer()
  #viz.setCameraFieldOfView( opensim.SimTK_PI / 3 )
  
  #viz.setCameraTransform( opensim.Transform( opensim.Vec3( 0, 0, 0.5 ) ) )
  #viz.setBackgroundType( viz.SolidColor )
  #viz.setBackgroundColor( opensim.White )
  
  positionErrorRMS = 0.0
  transparencyErrorRMS = 0.0
  timeSteps = [ 0.0 ]
  masterPositions = [ 0.0 ]
  slavePositions = [ 0.0 ]
  masterDelayedPositions = [ 0.0 ]
  slaveDelayedPositions = [ 0.0 ]
  masterPredictedPositions = [ 0.0 ]
  slavePredictedPositions = [ 0.0 ]
  masterInputs = [ 0.0 ]
  slaveInputs = [ 0.0 ]
  masterFeedbackInputs = [ 0.0 ]
  slaveFeedbackInputs = [ 0.0 ]
  masterInputInertias = [ 0.0 ]
  slaveInputInertias = [ 0.0 ]
  masterDelayedOutputInertias = [ 0.0 ]
  slaveDelayedOutputInertias = [ 0.0 ]
  masterInputEnergy = [ 0.0 ]
  slaveInputEnergy = [ 0.0 ]
  inputEnergy = [ 0.0 ]
  for timeStepIndex in range( 1, SIM_TIME_STEPS_NUMBER ):
    
    simTime = timeStepIndex * NET_TIME_STEP
    
    # general setpoints
    setpoint = setpoints[ timeStepIndex ]
    speedSetpoint = ( setpoints[ timeStepIndex ] - setpoints[ timeStepIndex - 1 ] ) / NET_TIME_STEP
    
    # master dynamics
    masterOutput = ( masterCoordinate.getValue( systemState ),
                     masterCoordinate.getSpeedValue( systemState ),
                     masterCoordinate.getAccelerationValue( systemState ) )
    masterInput = MASTER_KP * ( setpoint - masterOutput[ 0 ] ) + MASTER_KV * ( speedSetpoint - masterOutput[ 1 ] )
    masterInputActuator.setOverrideActuation( systemState, masterInput )
    # receive master delayed setpoints
    while simTime >= slaveToMasterTimesQueue[ 0 ]:
      slaveToMasterTimesQueue.pop( 0 )
      slaveDelayedOutput, slaveDelayedInput, slaveDelayedImpedance = slaveToMasterQueue.pop( 0 )
      if len( slaveToMasterQueue ) == 0: break
    # master control
    if USE_DYNAMIC_IMPEDANCE: masterTeleoperator.SetRemoteSystem( slaveDelayedImpedance )
    slaveFeedback = masterTeleoperator.Process( masterOutput, slaveDelayedOutput, slaveDelayedInput, slaveToMasterDelays[ -1 ] )
    slaveFeedbackInput, slavePredictedOutput, slaveCorrectedOutput = slaveFeedback
    masterFeedbackActuator.setOverrideActuation( systemState, slaveFeedbackInput )
    # linearize master system
    masterLinearizer.AddSample( masterOutput[ 0 ], masterOutput[ 1 ], masterOutput[ 2 ], masterInput, slaveFeedbackInput )
    masterInputImpedance, masterOutputImpedance = masterLinearizer.IdentifySystem( OPERATOR_IMPEDANCE )
    if USE_DYNAMIC_IMPEDANCE: masterTeleoperator.SetLocalSystem( masterOutputImpedance )
    # send slave delayed setpoints
    masterToSlaveQueue.append( ( masterOutput, masterInput, masterOutputImpedance ) )
    masterToSlaveDelays.append( NET_DELAY_AVG + NET_DELAY_VAR * random.randint( 0, 1000 ) / 1000.0 )
    masterToSlaveTimesQueue.append( simTime + masterToSlaveDelays[ -1 ] )

    # slave dynamics
    slaveOutput = ( slaveCoordinate.getValue( systemState ),
                    slaveCoordinate.getSpeedValue( systemState ),
                    slaveCoordinate.getAccelerationValue( systemState ) )
    slaveInput = 0.0
    if ENVIRONMENT_TYPE == 1: slaveInput = - SLAVE_KP * slaveOutput[ 0 ] - SLAVE_KV * slaveOutput[ 1 ]
    elif ENVIRONMENT_TYPE == 2: slaveInput = SLAVE_KP * ( setpoint - slaveOutput[ 0 ] ) + SLAVE_KV * ( speedSetpoint - slaveOutput[ 1 ] )
    elif ENVIRONMENT_TYPE == 3: slaveInput = SLAVE_KP * ( -setpoint - slaveOutput[ 0 ] ) + SLAVE_KV * ( -speedSetpoint - slaveOutput[ 1 ] )
    slaveInputActuator.setOverrideActuation( systemState, slaveInput )
    # receive slave delayed setpoints
    while simTime >= masterToSlaveTimesQueue[ 0 ]:
      masterToSlaveTimesQueue.pop( 0 )
      masterDelayedOutput, masterDelayedInput, masterDelayedImpedance = masterToSlaveQueue.pop( 0 )
      if len( masterToSlaveQueue ) == 0: break
    # slave control
    if USE_DYNAMIC_IMPEDANCE: slaveTeleoperator.SetRemoteSystem( masterDelayedImpedance )
    masterFeedback = slaveTeleoperator.Process( slaveOutput, masterDelayedOutput, masterDelayedInput, masterToSlaveDelays[ -1 ] )
    masterFeedbackInput, masterPredictedOutput, masterCorrectedOutput = masterFeedback
    slaveFeedbackActuator.setOverrideActuation( systemState, masterFeedbackInput )
    # linearize slave system
    slaveLinearizer.AddSample( slaveOutput[ 0 ], slaveOutput[ 1 ], slaveOutput[ 2 ], slaveInput, masterFeedbackInput )
    slaveInputImpedance, slaveOutputImpedance = slaveLinearizer.IdentifySystem( OPERATOR_IMPEDANCE )
    if USE_DYNAMIC_IMPEDANCE: slaveTeleoperator.SetLocalSystem( slaveOutputImpedance )
    # send master delayed setpoints
    slaveToMasterQueue.append( ( slaveOutput, slaveInput, slaveOutputImpedance ) )
    slaveToMasterDelays.append( NET_DELAY_AVG + NET_DELAY_VAR * random.randint( 0, 1000 ) / 1000.0 )
    slaveToMasterTimesQueue.append( simTime + slaveToMasterDelays[ -1 ] )
    
    # system update
    systemState = manager.integrate( simTime )
    
    # perfomance calculation
    positionErrorRMS += ( masterOutput[ 0 ] - slaveOutput[ 0 ] )**2 / SIM_TIME_STEPS_NUMBER
    transparencyErrorRMS += ( masterInputImpedance[ 0 ] - slaveDelayedImpedance[ 0 ] )**2 / SIM_TIME_STEPS_NUMBER
    transparencyErrorRMS += ( masterInputImpedance[ 1 ] - slaveDelayedImpedance[ 1 ] )**2 / SIM_TIME_STEPS_NUMBER
    transparencyErrorRMS += ( masterInputImpedance[ 2 ] - slaveDelayedImpedance[ 2 ] )**2 / SIM_TIME_STEPS_NUMBER
    transparencyErrorRMS += ( slaveInputImpedance[ 0 ] - masterDelayedImpedance[ 0 ] )**2 / SIM_TIME_STEPS_NUMBER
    transparencyErrorRMS += ( slaveInputImpedance[ 1 ] - masterDelayedImpedance[ 1 ] )**2 / SIM_TIME_STEPS_NUMBER
    transparencyErrorRMS += ( slaveInputImpedance[ 2 ] - masterDelayedImpedance[ 2 ] )**2 / SIM_TIME_STEPS_NUMBER
    
    # data logging
    timeSteps.append( simTime )
    masterPositions.append( masterOutput[ 0 ] )
    slavePositions.append( slaveOutput[ 0 ] )
    masterDelayedPositions.append( masterDelayedOutput[ 0 ] )
    slaveDelayedPositions.append( slaveDelayedOutput[ 0 ] )
    masterPredictedPositions.append( masterCorrectedOutput[ 0 ] )
    slavePredictedPositions.append( slaveCorrectedOutput[ 0 ] )
    masterInputs.append( masterInput )
    slaveInputs.append( slaveInput )
    masterFeedbackInputs.append( masterFeedbackInput )
    slaveFeedbackInputs.append( slaveFeedbackInput )
    masterInputInertias.append( masterInputImpedance[ 0 ] )
    slaveInputInertias.append( slaveInputImpedance[ 0 ] )
    masterDelayedOutputInertias.append( masterDelayedImpedance[ 0 ] )
    slaveDelayedOutputInertias.append( slaveDelayedImpedance[ 0 ] )
    masterInputPower = masterInput * masterOutput[ 1 ]
    masterInputEnergy.append( masterInputEnergy[ -1 ] + masterInputPower * NET_TIME_STEP )
    slaveInputPower = slaveInput * slaveOutput[ 1 ]
    slaveInputEnergy.append( slaveInputEnergy[ -1 ] + slaveInputPower * NET_TIME_STEP )
    inputEnergy.append( inputEnergy[ -1 ] + ( masterInputPower + slaveInputPower ) * NET_TIME_STEP )

  #model.setUseVisualizer( False )
  
  positionErrorRMS = math.sqrt( positionErrorRMS )
  transparencyErrorRMS = math.sqrt( transparencyErrorRMS )
  pyplot.subplot( 311, xlim=[ 0.0, SIM_TIME_STEPS_NUMBER * NET_TIME_STEP ], ylim=[ -0.2, 0.2 ] )
  pyplot.title( 'Teleoperation w/ {} Environment (delay={}±{}[s])\n{} Controller ( position RMS error={:.3f}, transparency RMS error={:.3f} )'.format( ENVIRONMENT_NAMES[ ENVIRONMENT_TYPE ], NET_DELAY_AVG, NET_DELAY_VAR, CONTROLLER_NAMES[ CONTROLLER_TYPE ], positionErrorRMS, transparencyErrorRMS ), fontsize=15 )
  pyplot.ylabel( 'Position [m]', fontsize=10 )
  pyplot.tick_params( axis='x', labelsize=0 )
  pyplot.plot( ( timeSteps[ 0 ], timeSteps[ -1 ] ), ( 0.0, 0.0 ), 'k--' )
  pyplot.plot( timeSteps, masterDelayedPositions, 'b:', timeSteps, slaveDelayedPositions, 'r:' )
  pyplot.plot( timeSteps, masterPredictedPositions, 'b--', timeSteps, slavePredictedPositions, 'r--' )
  pyplot.plot( timeSteps, masterPositions, 'b-', timeSteps, slavePositions, 'r-' )
  pyplot.legend( [ '0', 'master-delayed', 'slave-delayed', 'master-predicted', 'slave-predicted', 'master', 'slave' ] )
  pyplot.subplot( 312, xlim=[ 0.0, SIM_TIME_STEPS_NUMBER * NET_TIME_STEP ], ylim=[ -1.5, 1.5 ] )
  pyplot.ylabel( 'Force [N]', fontsize=10 )
  pyplot.tick_params( axis='x', labelsize=0 )
  pyplot.plot( ( timeSteps[ 0 ], timeSteps[ -1 ] ), ( 0.0, 0.0 ), 'k--' )
  pyplot.plot( timeSteps, masterInputs, 'g-', timeSteps, slaveInputs, 'm-' )
  pyplot.plot( timeSteps, masterFeedbackInputs, 'b-', timeSteps, slaveFeedbackInputs, 'r-' )
  pyplot.plot( timeSteps, masterInputInertias, 'g:', timeSteps, slaveInputInertias, 'm:' )
  pyplot.plot( timeSteps, slaveDelayedOutputInertias, 'b:', timeSteps, masterDelayedOutputInertias, 'r:' )
  pyplot.legend( [ '0', 'master-input', 'slave-input', 'master-feedback', 'slave-feedback' ] )
  pyplot.subplot( 313, xlim=[ 0.0, SIM_TIME_STEPS_NUMBER * NET_TIME_STEP ], ylim=[ -0.75, 0.75 ] )
  pyplot.ylabel( 'Energy [J]', fontsize=10 )
  pyplot.xlabel( 'Time [s]', fontsize=10 )
  pyplot.plot( ( timeSteps[ 0 ], timeSteps[ -1 ] ), ( 0.0, 0.0 ), 'k--' )
  pyplot.plot( timeSteps, masterInputEnergy, 'b-', timeSteps, slaveInputEnergy, 'r-', timeSteps, inputEnergy, 'g-' )
  #pyplot.plot( timeSteps, masterInputEnergy, 'g-' )
  pyplot.legend( [ '0', 'master-input', 'slave-input', 'net-input' ] )
  pyplot.show()
except Exception as e:
  print( e )
