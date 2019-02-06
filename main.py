import sys
import numpy

from lqg_control import GetLQGController 
from kalman_filter import KalmanFilter 

import opensim

from matplotlib import pyplot


data = numpy.loadtxt( sys.argv[ 2 ] )[ :10000 ]
samplingTime = data[ :, 0 ].ravel()
setpoint = data[ :, 1 ].ravel()

timeDelta = ( samplingTime[ -1 ] - samplingTime[ 0 ] ) / ( len(samplingTime) - 1 )

inertia, damping, weightFactor = LoadSystemParameters()
A = [ [ 1, timeDelta, timeDelta**2 / 2 ], [ 0, 1, timeDelta ], [ -weightFactor / inertia, -damping / inertia, 1 ] ]
B = [ [ 0 ], [ 0 ], [ -1 / inertia ] ]
C = [ [ 1, 0, 0 ] ]

feedbackGain = GetLQGController( A, B, C, 0.1 )

modelIDMLP = OSimIDMLP()
modelIDMLP.LoadWeights()
#modelEMGMLP = OSimEMGMLP()
#modelEMGMLP.LoadWeights()

inputObserver = KalmanFilter( 3, 1 )
inputObserver.SetMeasurement( 0, 0, 0.1 )
inputObserver.SetStatePredictionFactor( 0, 1, timeDelta )
inputObserver.SetStatePredictionFactor( 0, 2, ( timeDelta**2 ) / 2 )
inputObserver.SetStatePredictionFactor( 1, 2, timeDelta )

outputObserver = KalmanFilter( 3, 1, 1 )
outputObserver.SetMeasurement( 0, 0, 0.1 )
outputObserver.SetStatePredictionFactor( 0, 1, timeDelta )
outputObserver.SetStatePredictionFactor( 0, 2, ( timeDelta**2 ) / 2 )
outputObserver.SetStatePredictionFactor( 1, 2, timeDelta )
outputObserver.SetStatePredictionFactor( 2, 0, -weightFactor / inertia )
outputObserver.SetStatePredictionFactor( 2, 1, -damping / inertia )
outputObserver.SetInputPredictionFactor( 2, 0, -1 / inertia )

model = opensim.Model( sys.argv[ 1 ] )

model = opensim.Model()
bodyMaster = model.addBody( 'master', 1.0, opensim.Vec3(), opensim.Vec3() )
bodySlave = model.addBody( 'slave', 1.0, opensim.Vec3(), opensim.Vec3() )


model.setUseVisualizer( True )

tibiaMesh = opensim.WrapCylinder()
tibiaMesh.set_radius( 0.05 )
tibiaMesh.set_length( 0.5 )
tibiaMesh.set_translation( opensim.Vec3( 0, -0.25, 0 ) )
tibiaMesh.set_xyz_body_rotation( opensim.Vec3( opensim.SimTK_PI / 2, 0, 0 ) )
model.getBodySet().get( 'tibia_r' ).addWrapObject( tibiaMesh )
femurMesh = opensim.WrapCylinder()
femurMesh.set_radius( 0.05 )
femurMesh.set_length( 0.5 )
femurMesh.set_translation( opensim.Vec3( 0, -0.25, 0 ) )
femurMesh.set_xyz_body_rotation( opensim.Vec3( opensim.SimTK_PI / 2, 0, 0 ) )
model.getBodySet().get( 'femur_r' ).addWrapObject( femurMesh )

model.extendFinalizeFromProperties()

systemState = model.initSystem()
systemState.setTime( 0.0 )

actuator = model.getComponent( 'knee_actuator' )
actuator.overrideActuation( systemState, True )

#reporter = opensim.ConsoleReporter()
#reporter.addToReport( actuator.getCoordinate().getOutput( 'value' ), 'angle' )
#model.addComponent( reporter )

systemState = model.initSystem()
systemState.setTime( 0.0 )

musclesList = model.updMuscles()
for muscle in musclesList:
  muscle.setAppliesForce( systemState, False )

model.equilibrateMuscles( systemState )

manager = opensim.Manager( model )
manager.initialize( systemState )

viz = model.updVisualizer().updSimbodyVisualizer()
viz.setBackgroundType( viz.SolidColor )
viz.setBackgroundColor( opensim.White )

outputPosition = [ 0.0 ]
referenceInput = [ 0.0 ]
feedbackInput = [ 0.0 ]
userTorque = [ 0.0 ]
for timeStep in range( 1, samplingTime.size ):
#  timeDelta = samplingTime[ timeStep ] - samplingTime[ timeStep - 1 ]
    
  state, measures = inputObserver.Update( [ setpoint[ timeStep ] ] )
  setpointPosition = state[ 0 ]
  setpointVelocity = state[ 1 ]
  setpointAcceleration = state[ 2 ]

  referenceInput.append( modelIDMLP.Process( setpointPosition, setpointVelocity, setpointAcceleration, 0.0 )[ 0 ] )
  controlInput = feedbackInput[ timeStep - 1 ] + referenceInput[ timeStep - 1 ]

  # plant dynamics
  #actuator.setOverrideActuation( systemState, controlInput )
  manager = opensim.Manager( model )
  systemState.setTime( ( timeStep - 1 ) * timeDelta )
  manager.initialize( systemState )
  #model.realizeAcceleration( systemState )
  systemState = manager.integrate( timeStep * timeDelta )
  #model.realizeReport( systemState )
  print( actuator.getCoordinate().getValue( systemState ) )
  outputPosition.append( actuator.getCoordinate().getValue( systemState ) )
  outputVelocity = actuator.getCoordinate().getSpeedValue( systemState )
  outputAcceleration = actuator.getCoordinate().getAccelerationValue( systemState )
  
  state, measures = outputObserver.Update( [ outputPosition[ timeStep - 1 ] ], [ controlInput ] )  
  feedbackInput.append( -feedbackGain.dot( state )[ 0 ] )
  userTorque.append( modelIDMLP.Process( outputPosition[ -1 ], outputVelocity, outputAcceleration, controlInput )[ 0 ] )


model.setUseVisualizer( False )

plotTime = list( map( lambda t: t - samplingTime[ 0 ], samplingTime ) )
pyplot.plot( ( plotTime[ 0 ], plotTime[ -1 ] ), ( 0.0, 0.0 ), 'k--' )
pyplot.plot( plotTime, outputPosition, 'y-' )
#pyplot.plot( plotTime, setpoint, 'y--', plotTime, outputPosition, 'y-' )
#pyplot.plot( plotTime, referenceInput, 'r-', plotTime, feedbackInput, 'm-' )
pyplot.show()
