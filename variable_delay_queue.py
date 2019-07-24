from numpy import random, ravel, max, average, diff
from scipy.signal import butter, lfilter, freqz
from matplotlib import pyplot

from kalman_predictor import KalmanPredictor
from lqg_prediction_controller import LQGPredController as Controller

SIM_TIME_STEPS_NUMBER = 5000

NET_TIME_STEP = 0.02
NET_DELAY_AVG = 0.2
NET_DELAY_VAR = 0.1

remoteObserver = KalmanPredictor( NET_TIME_STEP )

random.seed( 0 )
setpoints = ravel( 2 * ( random.rand( 1, SIM_TIME_STEPS_NUMBER ) - 0.5 ) ).tolist()
b, a = butter( 2, 0.01, btype='low', analog=False )
setpoints = lfilter( b, a, setpoints )

setpointVelocities = diff( setpoints ) / NET_TIME_STEP
setpointAccelerations = diff( setpoints, n=2 ) / NET_TIME_STEP

masterToSlaveQueue = [ ( 0.0, 0.0, 0.0 ) ]
masterToSlaveTimesQueue = [ 0.0 ]
slaveToMasterQueue = [ ( 0.0, 0.0, 0.0 ) ]
slaveToMasterTimesQueue = [ 0.0 ]

timeSteps = [ 0.0 ]
masterOutputs = [ 0.0 ]
slaveInputs = [ 0.0 ]
slaveCorrectedInputs = [ 0.0 ]
slaveSmoothedInputs = [ 0.0 ]
masterToSlaveDelays = [ 0.0 ]
slaveToMasterDelays = [ 0.0 ]
slaveInput = ( 0.0, 0.0, 0.0 )
for timeStepIndex in range( 1, SIM_TIME_STEPS_NUMBER - 2 ):
    
  simTime = timeStepIndex * NET_TIME_STEP
  timeSteps.append( simTime )
  
#  if simTime >= masterToSlaveTimesQueue[ 0 ]:
  while simTime >= masterToSlaveTimesQueue[ 0 ]:
    masterToSlaveTimesQueue.pop( 0 )
    slaveInput = masterToSlaveQueue.pop( 0 )
    if len( masterToSlaveQueue ) == 0: break
  slaveInputs.append( slaveInput[ 0 ] )
    
  slaveCorrectedInput = remoteObserver.Process( slaveInput, simTime, masterToSlaveDelays[ -1 ] )
  slaveCorrectedInputs.append( slaveCorrectedInput[ 0 ] )
  
  #slaveSmoothedInput = remoteObserver.Smooth()
  #slaveSmoothedInputs.append( slaveSmoothedInput[ 0 ] )
  
  masterOutputs.append( setpoints[ timeStepIndex ] )
  masterToSlaveQueue.append( ( setpoints[ timeStepIndex ], setpointVelocities[ timeStepIndex ], setpointAccelerations[ timeStepIndex ] ) )
  masterToSlaveDelays.append( NET_DELAY_AVG + NET_DELAY_VAR * random.randint( 0, 1000 ) / 1000.0 )
  masterToSlaveTimesQueue.append( simTime + masterToSlaveDelays[ -1 ] )

#pyplot.subplot( 111, xlim=[ 0.0, SIM_TIME_STEPS_NUMBER * NET_TIME_STEP ], ylim=[ -0.2, 0.2 ] )
pyplot.plot( ( timeSteps[ 0 ], timeSteps[ -1 ] ), ( 0.0, 0.0 ), 'k--' )
pyplot.plot( timeSteps, masterToSlaveDelays, 'g-' )
pyplot.plot( timeSteps, masterOutputs, 'b-' )
pyplot.plot( timeSteps, slaveInputs, 'r--', timeSteps, slaveCorrectedInputs, 'r-' )
#pyplot.plot( timeSteps, slaveSmoothedInputs, 'm-' )
pyplot.show()
