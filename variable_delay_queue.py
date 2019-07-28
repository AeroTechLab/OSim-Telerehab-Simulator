from numpy import random, ravel, max, average, diff
from scipy.signal import butter, lfilter, freqz
from matplotlib import pyplot

from simple_plant import SimplePlant
from teleoperator import Teleoperator

SIM_TIME_STEPS_NUMBER = 5000

NET_TIME_STEP = 0.02
NET_DELAY_AVG = 0.2
NET_DELAY_VAR = 0.1

MASTER_KP = 10.0
MASTER_KV = 5.0

OPERATOR_INERTIA = 1.0
OPERATOR_DAMPING = 0.0
OPERATOR_STIFFNESS = 0.0
localPlant = SimplePlant( OPERATOR_INERTIA, OPERATOR_DAMPING, OPERATOR_STIFFNESS, NET_TIME_STEP )
localTeleoperator = Teleoperator( OPERATOR_INERTIA, OPERATOR_DAMPING, OPERATOR_STIFFNESS, NET_TIME_STEP )

random.seed( 0 )
setpoints = ravel( 2 * ( random.rand( 1, SIM_TIME_STEPS_NUMBER ) - 0.5 ) ).tolist()
b, a = butter( 2, 0.01, btype='low', analog=False )
setpoints = lfilter( b, a, setpoints )
setpointVelocities = diff( setpoints ) / NET_TIME_STEP
#setpointAccelerations = diff( setpoints, n=2 ) / NET_TIME_STEP

masterToSlaveQueue = [ ( ( 0.0, 0.0, 0.0 ), 0.0 ) ]
masterToSlaveTimesQueue = [ 0.0 ]
slaveToMasterQueue = [ ( ( 0.0, 0.0, 0.0 ), 0.0 ) ]
slaveToMasterTimesQueue = [ 0.0 ]

timeSteps = [ 0.0 ]
masterOutputs = [ 0.0 ]
slaveOutputs = [ 0.0 ]
slavePredictedOutputs = [ 0.0 ]
slaveCorrectedOutputs = [ 0.0 ]
masterToSlaveDelays = [ 0.0 ]
slaveToMasterDelays = [ 0.0 ]
masterOutput = ( 0.0, 0.0, 0.0 )
slaveInput = 0.0
slaveOutput = ( 0.0, 0.0, 0.0 )
slavePredictedOutput = ( 0.0, 0.0, 0.0 )
slaveCorrectedOutput = ( 0.0, 0.0, 0.0 )
slaveFeedback = 0.0
for timeStepIndex in range( 1, SIM_TIME_STEPS_NUMBER - 1 ):
    
  simTime = timeStepIndex * NET_TIME_STEP
  timeSteps.append( simTime )
  
  setpoint = setpoints[ timeStepIndex ]
  speedSetpoint = setpointVelocities[ timeStepIndex ]
  masterInput = MASTER_KP * ( setpoint - masterOutput[ 0 ] ) + MASTER_KV * ( speedSetpoint - masterOutput[ 1 ] )
  masterOutput = localPlant.Process( masterInput + slaveFeedback )
  
#  if simTime >= masterToSlaveTimesQueue[ 0 ]:
  while simTime >= masterToSlaveTimesQueue[ 0 ]:
    masterToSlaveTimesQueue.pop( 0 )
    slaveOutput, slaveInput = masterToSlaveQueue.pop( 0 )
    if len( masterToSlaveQueue ) == 0: break
  slaveOutputs.append( slaveOutput[ 0 ] )
    
  slaveFeedback, slavePredictedOutput, slaveCorrectedOutput = localTeleoperator.Process( masterOutput, slaveOutput, slaveInput, masterToSlaveDelays[ -1 ] )
    
  slavePredictedOutputs.append( slavePredictedOutput[ 0 ] )
  slaveCorrectedOutputs.append( slaveCorrectedOutput[ 0 ] )
  
  masterOutputs.append( masterOutput[ 0 ] )
  masterToSlaveQueue.append( ( masterOutput, masterInput ) )
  masterToSlaveDelays.append( NET_DELAY_AVG + NET_DELAY_VAR * random.randint( 0, 1000 ) / 1000.0 )
  masterToSlaveTimesQueue.append( simTime + masterToSlaveDelays[ -1 ] )

#pyplot.subplot( 111, xlim=[ 0.0, SIM_TIME_STEPS_NUMBER * NET_TIME_STEP ], ylim=[ -0.2, 0.2 ] )
pyplot.subplot( 111, xlim=[ 0.0, 5.0 ], ylim=[ -0.2, 0.2 ] )
pyplot.plot( ( timeSteps[ 0 ], timeSteps[ -1 ] ), ( 0.0, 0.0 ), 'k--' )
pyplot.plot( timeSteps, masterToSlaveDelays, 'g-' )
pyplot.plot( timeSteps, masterOutputs, 'b-' )
pyplot.plot( timeSteps, slaveOutputs, 'r--' )
pyplot.plot( timeSteps, slavePredictedOutputs, 'r-' )
pyplot.plot( timeSteps, slaveCorrectedOutputs, 'm-' )
pyplot.show()
