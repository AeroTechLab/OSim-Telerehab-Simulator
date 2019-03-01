from lqg_control import GetLQGController 
from kalman_filter import KalmanFilter 

class LQGPredController:
  state = [ 0.0, 0.0, 0.0 ]
  measurement = [ 0.0, 0.0, 0.0 ]
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
    self.remoteObserver = KalmanFilter( 3, 3 )
    self.remoteObserver.SetMeasurement( 0, 0, 1.0 )
    self.remoteObserver.SetMeasurement( 1, 1, 1.0 )
    self.remoteObserver.SetMeasurement( 1, 1, 1.0 )
    self.remoteObserver.SetPredictionNoise( 0, 2.0 )
    self.remoteObserver.SetPredictionNoise( 1, 2.0 )
    self.remoteObserver.SetPredictionNoise( 2, 2.0 )
  
  def PreProcess( self, inputPacket, timeDelay ):
    setpoint, setpointVelocity, setpointAcceleration = inputPacket
    remoteState = [ setpoint, setpointVelocity, setpointAcceleration ]
    timeDelay = int( timeDelay / self.dt ) * self.dt
    self.remoteObserver.SetStatePredictionFactor( 0, 1, timeDelay )
    self.remoteObserver.SetStatePredictionFactor( 0, 2, 0.5 * timeDelay**2 )
    self.remoteObserver.SetStatePredictionFactor( 1, 2, timeDelay )
    remoteState, self.measurement = self.remoteObserver.Predict( remoteState )
    if len( self.predictions ) > 0:
      if self.elapsedTime >= self.predictions[ 0 ][ 1 ]:
        predicition = self.predictions.pop( 0 )[ 0 ]
        self.remoteObserver.Update( self.measurement, predicition )
        self.remoteObserver.Correct()
    self.predictions.append( ( self.measurement, self.elapsedTime + timeDelay ) )
    self.elapsedTime += self.dt

  def Process( self, inputPosition, inputVelocity, inputForce ):
    reference = [ inputPosition - self.measurement[ 0 ], inputVelocity - self.measurement[ 1 ] ]
    self.state, measurement = self.localObserver.Process( reference, [ inputForce + self.outputForce ] )
    self.outputForce = -self.feedbackGain.dot( self.state )[ 0 ]
    return self.outputForce

  def PostProcess( self ):
    return ( self.state[ 0 ], self.state[ 1 ], self.state[ 2 ] )
