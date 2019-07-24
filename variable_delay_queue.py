from numpy import random, ravel, max, average
from scipy.signal import butter, lfilter, freqz
from matplotlib import pyplot

SIM_TIME_STEPS_NUMBER = 5000

NET_TIME_STEP = 0.02
NET_DELAY = 0.2

masterToSlaveQueue = [ ( 0.0, 0.0, 0.0 ) ]
masterToSlaveTimesQueue = [ 0.0 ]
slaveToMasterQueue = [ ( 0.0, 0.0, 0.0 ) ]
slaveToMasterTimesQueue = [ 0.0 ]

random.seed( 0 )
setpoints = ravel( 2 * ( random.rand( 1, SIM_TIME_STEPS_NUMBER ) - 0.5 ) ).tolist()
b, a = butter( 2, 0.01, btype='low', analog=False )
setpoints = lfilter( b, a, setpoints )

timeSteps = [ 0.0 ]
masterOutputs = [ 0.0 ]
slaveInputs = [ 0.0 ]

for timeStepIndex in range( 1, SIM_TIME_STEPS_NUMBER ):
    
  simTime = timeStepIndex * NET_TIME_STEP
  timeSteps.append( simTime )
  
  slaveInput = slaveInputs[ -1 ]
  if simTime > masterToSlaveTimesQueue[ 0 ]:
    masterToSlaveTimesQueue.pop( 0 )
    slaveInput = masterToSlaveQueue.pop( 0 )[ 0 ]
  slaveInputs.append( slaveInput )
  
  masterOutputs.append( setpoints[ timeStepIndex ] )
  masterToSlaveQueue.append( ( masterOutputs[ -1 ], 0.0, 0.0 ) )
  masterToSlaveTimesQueue.append( simTime + NET_DELAY + NET_TIME_STEP * random.randint( -1, 1 ) )

pyplot.plot( ( timeSteps[ 0 ], timeSteps[ -1 ] ), ( 0.0, 0.0 ), 'k--' )
pyplot.plot( timeSteps, masterOutputs, 'b-', timeSteps, slaveInputs, 'r-' )
pyplot.show()
