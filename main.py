import math

#from wave_controller import WaveController as Controller
#from pid_controller import PIDController as Controller
#from nn_controller import NNController as Controller
#from lqg_controller import LQGController as Controller

#from pv_controller import PVController as Controller
#from wave_position_controller import WaveController as Controller
from lqg_prediction_controller import LQGPredController as Controller

import opensim

from numpy import random, ravel, max, average
from scipy.signal import butter, lfilter, freqz
from matplotlib import pyplot

SIM_TIME_STEPS_NUMBER = 5000

MASTER_KP = 10.0
MASTER_KV = 1.0
SLAVE_KP = 5.0
SLAVE_KV = 0.5

NET_TIME_STEP = 0.02
NET_DELAY = 0.2
netDataQueueLength = int( NET_DELAY / NET_TIME_STEP )
masterToSlaveQueue = [ ( 0.0, 0.0, 0.0 ) for packet in range( netDataQueueLength ) ]
slaveToMasterQueue = [ ( 0.0, 0.0, 0.0 ) for packet in range( netDataQueueLength ) ]

masterController = Controller( NET_TIME_STEP )
slaveController = Controller( NET_TIME_STEP )

random.seed( 0 )
setpoints = ravel( 2 * ( random.rand( 1, SIM_TIME_STEPS_NUMBER ) - 0.5 ) ).tolist()
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
  
  errorRMS = 0.0
  timeSteps = [ 0.0 ]
  masterPositions = [ 0.0 ]
  slavePositions = [ 0.0 ]
  masterForces = [ 0.0 ]
  slaveForces = [ 0.0 ]
  inputEnergy = [ 0.0 ]
  outputEnergy = [ 0.0 ]
  storedEnergy = [ 0.0 ]
  for timeStepIndex in range( 1, SIM_TIME_STEPS_NUMBER ):
    
    simTime = timeStepIndex * NET_TIME_STEP
    
    dataIndex = timeStepIndex % netDataQueueLength

    #plant dynamics
    
    setpoint = setpoints[ timeStepIndex ]#math.sin( 2 * math.pi * simTime / 4 )
    speedSetpoint = ( setpoints[ timeStepIndex ] - setpoints[ timeStepIndex - 1 ] ) / NET_TIME_STEP#math.cos( 2 * math.pi * simTime / 4 )
    
    masterPosition = masterCoordinate.getValue( systemState )
    masterSpeed = masterCoordinate.getSpeedValue( systemState )
    #masterAcceleration = masterCoordinate.getAccelerationValue( systemState )
    masterInput = MASTER_KP * ( setpoint - masterPosition ) + MASTER_KV * ( speedSetpoint - masterSpeed )
    masterInputActuator.setOverrideActuation( systemState, masterInput )
    masterController.PreProcess( slaveToMasterQueue[ dataIndex ], NET_DELAY )
    slaveFeedback = masterController.Process( masterPosition, masterSpeed, masterInput )
    masterFeedbackActuator.setOverrideActuation( systemState, slaveFeedback )
    
    slavePosition = slaveCoordinate.getValue( systemState )
    slaveSpeed = slaveCoordinate.getSpeedValue( systemState )
    #slaveAcceleration = slaveCoordinate.getAccelerationValue( systemState )
    slaveController.PreProcess( masterToSlaveQueue[ dataIndex ], NET_DELAY )
    #slaveInput = - SLAVE_KP * slavePosition - SLAVE_KV * slaveSpeed
    #slaveInput = SLAVE_KP * ( setpoint - slavePosition ) + SLAVE_KV * ( speedSetpoint - slaveSpeed )
    slaveInput = SLAVE_KP * ( -setpoint - slavePosition ) + SLAVE_KV * ( -speedSetpoint - slaveSpeed )
    slaveInputActuator.setOverrideActuation( systemState, slaveInput )
    masterFeedback = slaveController.Process( slavePosition, slaveSpeed, slaveInput )
    slaveFeedbackActuator.setOverrideActuation( systemState, masterFeedback )
    
    masterToSlaveQueue[ dataIndex ] = masterController.PostProcess()
    slaveToMasterQueue[ dataIndex ] = slaveController.PostProcess()

    systemState = manager.integrate( simTime )
    
    errorRMS += ( masterPosition - slavePosition )**2 / SIM_TIME_STEPS_NUMBER
    timeSteps.append( simTime )
    masterPositions.append( masterPosition )
    slavePositions.append( slavePosition )
    masterForces.append( slaveFeedback )
    slaveForces.append( masterFeedback )
    inputEnergy.append( inputEnergy[ -1 ] + masterInput * masterSpeed * NET_TIME_STEP )
    outputEnergy.append( outputEnergy[ -1 ] + slaveInput * slaveSpeed * NET_TIME_STEP )

  #model.setUseVisualizer( False )
  
  print( 'RMS error:', math.sqrt( errorRMS ) )
  pyplot.subplot( 311, xlim=[ 0.0, 100.0 ], ylim=[ -0.3, 0.3 ], ylabel='Position [m]' )
  pyplot.tick_params( axis='x', labelsize=0 )
  pyplot.plot( ( timeSteps[ 0 ], timeSteps[ -1 ] ), ( 0.0, 0.0 ), 'k--' )
  pyplot.plot( timeSteps, masterPositions, 'b-', timeSteps, slavePositions, 'r-' )
  #pyplot.legend( [ '', 'master', 'slave' ] )
  pyplot.subplot( 312, xlim=[ 0.0, 100.0 ], ylim=[ -1.5, 1.5 ], ylabel='Force [N]' )
  pyplot.tick_params( axis='x', labelsize=0 )
  pyplot.plot( ( timeSteps[ 0 ], timeSteps[ -1 ] ), ( 0.0, 0.0 ), 'k--' )
  pyplot.plot( timeSteps, masterForces, 'b-', timeSteps, slaveForces, 'r-' )
  pyplot.subplot( 313, xlim=[ 0.0, 100.0 ], ylim=[ -1.0, 1.0 ], ylabel='Energy [J]' )
  pyplot.plot( ( timeSteps[ 0 ], timeSteps[ -1 ] ), ( 0.0, 0.0 ), 'k--' )
  pyplot.plot( timeSteps, inputEnergy, 'g-', timeSteps, outputEnergy, 'm-' )
  #pyplot.plot( timeSteps, inputEnergy, 'g-' )
  pyplot.show()
except Exception as e:
  print( e )
