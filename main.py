#import sys
#import numpy
import math

#from wave_controller import WaveController as Controller
#from pid_controller import PIDController as Controller
#from nn_controller import NNController as Controller
#from lqg_controller import LQGController as Controller
from  pv_controller import PVController as Controller

import opensim

from matplotlib import pyplot


#data = numpy.loadtxt( sys.argv[ 2 ] )[ :10000 ]
#samplingTime = data[ :, 0 ].ravel()
#setpoint = data[ :, 1 ].ravel()

#timeDelta = ( samplingTime[ -1 ] - samplingTime[ 0 ] ) / ( len(samplingTime) - 1 )

#inertia, damping, weightFactor = LoadSystemParameters()
#A = [ [ 1, timeDelta, timeDelta**2 / 2 ], [ 0, 1, timeDelta ], [ -weightFactor / inertia, -damping / inertia, 1 ] ]
#B = [ [ 0 ], [ 0 ], [ -1 / inertia ] ]
#C = [ [ 1, 0, 0 ] ]

#feedbackGain = GetLQGController( A, B, C, 0.1 )

#modelIDMLP = OSimIDMLP()
#modelIDMLP.LoadWeights()
##modelEMGMLP = OSimEMGMLP()
##modelEMGMLP.LoadWeights()

#inputObserver = KalmanFilter( 3, 1 )
#inputObserver.SetMeasurement( 0, 0, 0.1 )
#inputObserver.SetStatePredictionFactor( 0, 1, timeDelta )
#inputObserver.SetStatePredictionFactor( 0, 2, ( timeDelta**2 ) / 2 )
#inputObserver.SetStatePredictionFactor( 1, 2, timeDelta )

#outputObserver = KalmanFilter( 3, 1, 1 )
#outputObserver.SetMeasurement( 0, 0, 0.1 )
#outputObserver.SetStatePredictionFactor( 0, 1, timeDelta )
#outputObserver.SetStatePredictionFactor( 0, 2, ( timeDelta**2 ) / 2 )
#outputObserver.SetStatePredictionFactor( 1, 2, timeDelta )
#outputObserver.SetStatePredictionFactor( 2, 0, -weightFactor / inertia )
#outputObserver.SetStatePredictionFactor( 2, 1, -damping / inertia )
#outputObserver.SetInputPredictionFactor( 2, 0, -1 / inertia )

SIM_TIME_STEPS_NUMBER = 1000

CONTROLLER_KP = 10.0
CONTROLLER_KD = 1.0

NET_TIME_STEP = 0.02
NET_DELAY = 0.1
netDataQueueLength = int( NET_DELAY / NET_TIME_STEP )
masterToSlaveQueue = [ ( 0.0, 0.0 ) for packet in range( netDataQueueLength ) ]
slaveToMasterQueue = [ ( 0.0, 0.0 ) for packet in range( netDataQueueLength ) ]

masterController = Controller()
slaveController = Controller()

try:
  model = opensim.Model()
  
  model.setUseVisualizer( True )
  
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
  
  #reporter = opensim.ConsoleReporter()
  #reporter.addToReport( masterActuator.getCoordinate().getOutput( 'value' ), 'angle' )
  #model.addComponent( reporter )
  
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

  viz = model.updVisualizer().updSimbodyVisualizer()
  viz.setCameraFieldOfView( opensim.SimTK_PI / 3 )
  #viz.setCameraTransform( opensim.Transform( opensim.Vec3( 0, 0, 0.5 ) ) )
  #viz.setBackgroundType( viz.SolidColor )
  #viz.setBackgroundColor( opensim.White )
  
  timeSteps = [ 0.0 ]
  masterPositions = [ 0.0 ]
  slavePositions = [ 0.0 ]
  masterForces = [ 0.0 ]
  slaveForces = [ 0.0 ]
  for timeStepIndex in range( 1, SIM_TIME_STEPS_NUMBER ):
  #timeDelta = samplingTime[ timeStep ] - samplingTime[ timeStep - 1 ]
    
    simTime = timeStepIndex * NET_TIME_STEP
    
    dataIndex = timeStepIndex % netDataQueueLength
    
    #state, measures = inputObserver.Update( [ setpoint[ timeStep ] ] )
    #setpointPosition = state[ 0 ]
    #setpointVelocity = state[ 1 ]
    #setpointAcceleration = state[ 2 ]

    #referenceInput.append( modelIDMLP.Process( setpointPosition, setpointVelocity, setpointAcceleration, 0.0 )[ 0 ] )
    #controlInput = feedbackInput[ timeStep - 1 ] + referenceInput[ timeStep - 1 ]

    #plant dynamics
    #masterActuator.setOverrideActuation( systemState, controlInput )
    
    masterSetpoint = math.sin( 2 * math.pi * simTime / 4 )
    masterSpeedSetpoint = math.cos( 2 * math.pi * simTime / 4 )
    masterPosition = masterCoordinate.getValue( systemState )
    masterSpeed = masterCoordinate.getSpeedValue( systemState )
    masterInput = CONTROLLER_KP * ( masterSetpoint - masterPosition ) + CONTROLLER_KD * ( masterSpeedSetpoint - masterSpeed )
    masterInputActuator.setOverrideActuation( systemState, masterInput )
    masterController.PreProcess( slaveToMasterQueue[ dataIndex ], NET_DELAY, NET_TIME_STEP )
    slaveFeedback = masterController.Process( masterPosition, masterSpeed, masterInput )
    masterFeedbackActuator.setOverrideActuation( systemState, slaveFeedback )
    
    slavePosition = slaveCoordinate.getValue( systemState )
    slaveSpeed = slaveCoordinate.getSpeedValue( systemState )
    slaveController.PreProcess( masterToSlaveQueue[ dataIndex ], NET_DELAY, NET_TIME_STEP )
    masterFeedback = slaveController.Process( slavePosition, slaveSpeed, 0.0 )
    slaveFeedbackActuator.setOverrideActuation( systemState, masterFeedback )
    
    masterToSlaveQueue[ dataIndex ] = masterController.PostProcess()
    slaveToMasterQueue[ dataIndex ] = slaveController.PostProcess()
    
    #model.realizeAcceleration( systemState )
    
    systemState = manager.integrate( simTime )
    
    #model.realizeReport( systemState )
    
    timeSteps.append( simTime )
    masterPositions.append( masterPosition )
    slavePositions.append( slavePosition )
    masterForces.append( slaveFeedback )
    slaveForces.append( masterFeedback )
    
    #print( masterActuator.getCoordinate().getValue( systemState ) )
    #outputPosition.append( actuator.getCoordinate().getValue( systemState ) )
    #outputVelocity = actuator.getCoordinate().getSpeedValue( systemState )
    #outputAcceleration = actuator.getCoordinate().getAccelerationValue( systemState )
    
    #state, measures = outputObserver.Update( [ outputPosition[ timeStep - 1 ] ], [ controlInput ] )  
    #feedbackInput.append( -feedbackGain.dot( state )[ 0 ] )
    #userTorque.append( modelIDMLP.Process( outputPosition[ -1 ], outputVelocity, outputAcceleration, controlInput )[ 0 ] )


  model.setUseVisualizer( False )

  pyplot.plot( ( timeSteps[ 0 ], timeSteps[ -1 ] ), ( 0.0, 0.0 ), 'k--' )
  #pyplot.plot( timeSteps, outputPosition, 'y-' )
  pyplot.plot( timeSteps, masterPositions, 'b--', timeSteps, slavePositions, 'b-' )
  pyplot.plot( timeSteps, masterForces, 'r-', timeSteps, slaveForces, 'm-' )
  pyplot.show()
except Exception as e:
  print( e )
