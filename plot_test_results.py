import numpy

import data_plotter
from system_linearizer import SystemLinearizer

SAMPLES_NUMBER = int( 60 / 0.02 )

clientLinearizer = SystemLinearizer()

clientData = numpy.loadtxt( 'client Move Ball-lqg-2.log' )
serverData = numpy.loadtxt( 'server Move Ball-lqg-2.log' )
minTimeStep = max( clientData[ 0, 0 ], serverData[ 0, 0 ] )
maxTimeStep = min( clientData[ -1, 0 ], serverData[ -1, 0 ] )

clientTimeSteps = []
clientPositions = []; clientVelocities = []
clientDelayedPositions = []; clientSetpointPositions = []
clientInputs = []; serverFeedbackInputs = []
clientInputInertias = []; clientInputDampings = []; clientInputStiffnesses = []
clientInputEnergy = []; serverFeedbackEnergy = []; clientDissipatedEnergy = []
clientNetEnergy = [];

clientDampings = []

inputEnergy = 0.0; feedbackEnergy = 0.0; dissipatedEnergy = 0.0

index = 0
samplesCount = 0
while samplesCount < SAMPLES_NUMBER:
  if minTimeStep < clientData[ index, 0 ] < maxTimeStep: 
    clientTimeSteps.append( clientData[ index, 0 ] - minTimeStep )
    timeDelta = clientData[ index + 1, 0 ] - clientData[ index, 0 ]
    clientSetpointPositions.append( clientData[ index, 1 ] )
    position = clientData[ index, 2 ]
    velocity = clientData[ index, 3 ]
    clientPositions.append( position )
    clientVelocities.append( velocity )
    acceleration = clientData[ index, 4 ]
    inputForce = clientData[ index, 5 ]
    clientInputs.append( inputForce )
    feedbackForce = clientData[ index, 6 ]
    serverFeedbackInputs.append( feedbackForce )
    clientLinearizer.AddSample( position, velocity, acceleration, inputForce, feedbackForce )
    inputImpedance, outputImpedance, plantImpedance = clientLinearizer.IdentifySystem( ( 0.0, 0.0, 0.0 ) )
    clientInputInertias.append( inputImpedance[ 0 ] )
    clientInputDampings.append( inputImpedance[ 1 ] )
    clientInputStiffnesses.append( inputImpedance[ 2 ] )
    inertia = clientData[ index, 7 ]
    damping = clientData[ index, 8 ]
    clientDampings.append( damping )
    stiffness = clientData[ index, 9 ]
    inputEnergy += inputForce * velocity * timeDelta
    clientInputEnergy.append( inputEnergy )
    feedbackEnergy += feedbackForce * velocity * timeDelta
    serverFeedbackEnergy.append( feedbackEnergy )
    dissipatedEnergy -= plantImpedance[ 1 ] * velocity * velocity * timeDelta
    clientDissipatedEnergy.append( dissipatedEnergy )
    clientNetEnergy.append( plantImpedance[ 0 ] * velocity * velocity / 2 )
    samplesCount += 1
  index += 1
  
print( numpy.mean( numpy.array( clientDampings ), axis=0 ) )


serverLinearizer = SystemLinearizer()
  
serverTimeSteps = []
serverPositions = []; serverVelocities = []
serverDelayedPositions = []; serverSetpointPositions = []
serverInputs = []; clientFeedbackInputs = []
serverOutputInertias = []; serverOutputDampings = []; serverOutputStiffnesses = []

index = 0
samplesCount = 0
while samplesCount < SAMPLES_NUMBER:
  if minTimeStep < serverData[ index, 0 ] < maxTimeStep: 
    serverTimeSteps.append( serverData[ index, 0 ] - minTimeStep )
    timeDelta = serverData[ index + 1, 0 ] - serverData[ index, 0 ]
    serverSetpointPositions.append( serverData[ index, 1 ] )
    position = serverData[ index, 2 ]
    velocity = serverData[ index, 3 ]
    serverPositions.append( position )
    serverVelocities.append( velocity )
    acceleration = serverData[ index, 4 ]
    inputForce = serverData[ index, 5 ]
    serverInputs.append( inputForce )
    feedbackForce = serverData[ index, 6 ]
    clientFeedbackInputs.append( feedbackForce )
    serverLinearizer.AddSample( position, velocity, acceleration, inputForce, feedbackForce )
    inputImpedance, outputImpedance, plantImpedance = serverLinearizer.IdentifySystem( ( 0.0, 0.0, 0.0 ) )
    serverOutputInertias.append( outputImpedance[ 0 ] )
    serverOutputDampings.append( outputImpedance[ 1 ] )
    serverOutputStiffnesses.append( outputImpedance[ 2 ] )
    samplesCount += 1
  index += 1
  
timeSteps = numpy.mean( numpy.array( [ clientTimeSteps, serverTimeSteps ] ), axis=0 ).tolist()
referencePositions = [ 0.0 for step in range( len( timeSteps ) ) ]
referenceEnergy = [ 0.0 for step in range( len( timeSteps ) ) ]
clientDelayedPositions = [ 0.0 for step in range( len( timeSteps ) ) ]
clientSetpointPositions = [ 0.0 for step in range( len( timeSteps ) ) ]
serverDelayedPositions = [ 0.0 for step in range( len( timeSteps ) ) ]
serverSetpointPositions = [ 0.0 for step in range( len( timeSteps ) ) ]

data_plotter.plotData( timeSteps, referencePositions, clientPositions, serverPositions, clientVelocities, serverVelocities, 
                       clientDelayedPositions, serverDelayedPositions, clientSetpointPositions, serverSetpointPositions, 
                       clientInputs, serverInputs, clientFeedbackInputs, serverFeedbackInputs, clientInputInertias, serverOutputInertias, 
                       clientInputDampings, serverOutputDampings, clientInputStiffnesses, serverOutputStiffnesses, clientInputEnergy, 
                       serverFeedbackEnergy, clientNetEnergy, clientDissipatedEnergy, referenceEnergy )
