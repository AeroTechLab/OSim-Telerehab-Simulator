from lqg_control import GetLQGController 
from kalman_filter import KalmanFilter 

class LQGPredController:
  state = [ 0.0, 0.0, 0.0 ]
  remoteState = [ 0.0, 0.0, 0.0 ]
  reference = [ 0.0, 0.0 ]
  setpoint = 0.0
  setpointVelocity = 0.0
  predictions = []
  elapsedTime = 0.0
  outputForce = 0.0
  
  def __init__( self, timeDelta ):
    self.dt = timeDelta
    A = [ [ 1, self.dt, 0.5 * self.dt**2 ], [ 0, 1, self.dt ], [ 0, 0, 0 ] ]
    B = [ [ 0 ], [ 0 ], [ 1 / 1.0 ] ]
    C = [ [ 1, 0, 0 ], [ 0, 1, 0 ], [ 0, 0, 1 ] ]
    self.feedbackGain = GetLQGController( A, B, C, 0.0001 )
    self.localObserver = KalmanFilter( 3, 2 )
    self.localObserver.SetMeasurement( 0, 0, 1.0 )
    self.localObserver.SetMeasurement( 1, 1, 1.0 )
    self.localObserver.SetStatePredictionFactor( 0, 1, A[ 0 ][ 1 ] )
    self.localObserver.SetStatePredictionFactor( 0, 2, A[ 0 ][ 2 ] )
    self.localObserver.SetStatePredictionFactor( 1, 2, A[ 1 ][ 2 ] )
    self.localObserver.SetStatePredictionFactor( 2, 2, A[ 2 ][ 2 ] )
    self.localObserver.SetInputPredictionFactor( 2, 0, B[ 2 ][ 0 ] )
    self.remoteObserver = KalmanFilter( 3, 2 )
    self.remoteObserver.SetMeasurement( 0, 0, 1.0 )
    self.remoteObserver.SetMeasurement( 1, 1, 1.0 )
    self.remoteObserver.SetPredictionNoise( 0, 2.0 )
    self.remoteObserver.SetPredictionNoise( 1, 2.0 )
    self.remoteObserver.SetPredictionNoise( 2, 2.0 )
  
  def PreProcess( self, inputPacket, timeDelay ):
    self.setpoint, self.setpointVelocity = inputPacket
    self.remoteState[ 0 ] = self.setpoint
    self.remoteState[ 1 ] = self.setpointVelocity
    self.remoteObserver.SetStatePredictionFactor( 0, 1, timeDelay )
    self.remoteObserver.SetStatePredictionFactor( 0, 2, 0.5 * timeDelay**2 )
    self.remoteObserver.SetStatePredictionFactor( 1, 2, timeDelay )
    self.remoteState, self.reference = self.remoteObserver.Predict( self.remoteState )
    if len( self.predictions ) > 0:
      if self.elapsedTime >= self.predictions[ 0 ][ 1 ]:
        predicition = self.predictions.pop( 0 )[ 0 ]
        self.remoteObserver.Update( self.reference, predicition )
        self.remoteObserver.Correct()
    self.predictions.append( ( self.reference, self.elapsedTime + timeDelay ) )
    self.elapsedTime += self.dt

  def Process( self, inputPosition, inputVelocity, inputForce ):
    self.reference[ 0 ] = inputPosition - self.reference[ 0 ]
    self.reference[ 1 ] = inputVelocity - self.reference[ 1 ]
    self.state, measurement = self.localObserver.Process( self.reference, [ inputForce + self.outputForce ] )
    self.outputForce = -self.feedbackGain.dot( self.state )[ 0 ]
    return self.outputForce

  def PostProcess( self ):
    return ( self.state[ 0 ], self.state[ 1 ] )
