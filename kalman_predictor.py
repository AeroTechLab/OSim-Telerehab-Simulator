from kalman_filter import KalmanFilter 

class KalmanPredictor:
  predictions = []
  
  def __init__( self, timeStep ):
    self.dt = timeStep
    self.remoteObserver = KalmanFilter( 3, 3 )
    self.remoteObserver.SetMeasurement( 0, 0, 1.0 )
    self.remoteObserver.SetMeasurement( 1, 1, 1.0 )
    self.remoteObserver.SetMeasurement( 2, 2, 1.0 )
    self.remoteObserver.SetPredictionNoise( 0, 2.0 )
    self.remoteObserver.SetPredictionNoise( 1, 2.0 )
    self.remoteObserver.SetPredictionNoise( 2, 2.0 )
  
  def Process( self, inputPacket, remoteTime, timeDelay ):
    setpoint, setpointVelocity, setpointAcceleration = inputPacket
    measurement = [ setpoint, setpointVelocity, setpointAcceleration ]
    timeDelay = int( timeDelay / self.dt ) * self.dt
    self.remoteObserver.SetStatePredictionFactor( 0, 1, timeDelay )
    self.remoteObserver.SetStatePredictionFactor( 0, 2, 0.5 * timeDelay**2 )
    self.remoteObserver.SetStatePredictionFactor( 1, 2, timeDelay )
    state, prediction = self.remoteObserver.Predict( measurement )
    if len( self.predictions ) > 0:
      if remoteTime >= self.predictions[ 0 ][ 1 ]:
        estimatedMeasurement = self.predictions.pop( 0 )[ 0 ]
        self.remoteObserver.Update( measurement, estimatedMeasurement )
        self.remoteObserver.Correct()
    self.predictions.append( ( prediction, remoteTime + timeDelay ) )
    return [ prediction[ 0 ], prediction[ 1 ], prediction[ 2 ] ]

