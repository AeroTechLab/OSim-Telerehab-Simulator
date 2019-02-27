import math

#from wave_controller import WaveController as Controller
#from pid_controller import PIDController as Controller
#from nn_controller import NNController as Controller
#from lqg_controller import LQGController as Controller
#from lqg_prediction_controller import LQGPredController as Controller
from  pv_controller import PVController as Controller

import opensim

from numpy import random, ravel
from scipy.signal import butter, lfilter, freqz
from matplotlib import pyplot

SIM_TIME_STEPS_NUMBER = 10000

CONTROLLER_KP = 10.0
CONTROLLER_KD = 1.0

NET_TIME_STEP = 0.02
NET_DELAY = 0.1
netDataQueueLength = int( NET_DELAY / NET_TIME_STEP )
masterToSlaveQueue = [ ( 0.0, 0.0 ) for packet in range( netDataQueueLength ) ]
slaveToMasterQueue = [ ( 0.0, 0.0 ) for packet in range( netDataQueueLength ) ]

masterController = Controller( NET_TIME_STEP )
slaveController = Controller( NET_TIME_STEP )

random.seed( 0 )
setpoints = ravel( 4 * ( random.rand( 1, SIM_TIME_STEPS_NUMBER ) - 0.5 ) ).tolist()
b, a = butter( 2, 0.01, btype='low', analog=False )
setpoints = lfilter( b, a, setpoints )

try:
  model = opensim.Model()
  
  #model.setUseVisualizer( True )
  
  model.setName( 'TelerehabSimulator' )
  model.setGravity( opensim.Vec3( 0, 0, 0 ) )

  master = opensim.Body( 'master', 1.0, opensim.Vec3( 0, 0, 0 ), opensim.Inertia( 0, 0, 0 ) )
  model.addBody( master )
  slave = opensim.Body( 'slave', 1.0, opensim.Vec3( 0, 0, 0 ), opensim.Inertia( 0, 0, 0 ) )
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
  
  errorAbsSum = 0.0
  masterEnergy = 0.0
  slaveEnergy = 0.0
  timeSteps = [ 0.0 ]
  masterPositions = [ 0.0 ]
  slavePositions = [ 0.0 ]
  masterForces = [ 0.0 ]
  slaveForces = [ 0.0 ]
  for timeStepIndex in range( 1, SIM_TIME_STEPS_NUMBER ):
    
    simTime = timeStepIndex * NET_TIME_STEP
    
    dataIndex = timeStepIndex % netDataQueueLength

    #plant dynamics
    
    masterSetpoint = setpoints[ timeStepIndex ]#math.sin( 2 * math.pi * simTime / 4 )
    masterSpeedSetpoint = 0.0#( setpoints[ timeStepIndex ] - setpoints[ timeStepIndex - 1 ] ) / NET_TIME_STEP#math.cos( 2 * math.pi * simTime / 4 )
    masterPosition = masterCoordinate.getValue( systemState )
    masterSpeed = masterCoordinate.getSpeedValue( systemState )
    #masterAcceleration = masterCoordinate.getAccelerationValue( systemState )
    masterInput = CONTROLLER_KP * ( masterSetpoint - masterPosition ) + CONTROLLER_KD * ( masterSpeedSetpoint - masterSpeed )
    masterInputActuator.setOverrideActuation( systemState, masterInput )
    masterController.PreProcess( slaveToMasterQueue[ dataIndex ], NET_DELAY )
    slaveFeedback = masterController.Process( masterPosition, masterSpeed, masterInput )
    masterFeedbackActuator.setOverrideActuation( systemState, slaveFeedback )
    masterEnergy += abs( ( masterInput + slaveFeedback ) * masterSpeed )
    
    slavePosition = slaveCoordinate.getValue( systemState )
    slaveSpeed = slaveCoordinate.getSpeedValue( systemState )
    #slaveAcceleration = slaveCoordinate.getAccelerationValue( systemState )
    slaveController.PreProcess( masterToSlaveQueue[ dataIndex ], NET_DELAY )
    slaveInput = 0.0#-slavePosition
    slaveInputActuator.setOverrideActuation( systemState, slaveInput )
    masterFeedback = slaveController.Process( slavePosition, slaveSpeed, slaveInput )
    slaveFeedbackActuator.setOverrideActuation( systemState, masterFeedback )
    slaveEnergy += abs( ( slaveInput + masterFeedback ) * slaveSpeed )
    
    masterToSlaveQueue[ dataIndex ] = masterController.PostProcess()
    slaveToMasterQueue[ dataIndex ] = slaveController.PostProcess()

    systemState = manager.integrate( simTime )
    
    errorAbsSum += abs( masterPosition - slavePosition )
    timeSteps.append( simTime )
    masterPositions.append( masterPosition )
    slavePositions.append( slavePosition )
    masterForces.append( slaveFeedback )
    slaveForces.append( masterFeedback )

  #model.setUseVisualizer( False )
  
  print( 'error:', errorAbsSum )
  print( 'energy:', masterEnergy, slaveEnergy )
  pyplot.plot( ( timeSteps[ 0 ], timeSteps[ -1 ] ), ( 0.0, 0.0 ), 'k--' )
  #pyplot.plot( timeSteps, outputPosition, 'y-' )
  pyplot.plot( timeSteps, masterPositions, 'b--', timeSteps, slavePositions, 'b-' )
  pyplot.plot( timeSteps, masterForces, 'r-', timeSteps, slaveForces, 'm-' )
  pyplot.show()
except Exception as e:
  print( e )
