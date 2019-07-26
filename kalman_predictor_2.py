from kalman_filter import KalmanFilter 

class KalmanPredictor:
  state = [ 0.0, 0.0, 0.0 ]
  
  def __init__( self, timeStep ):
    self.dt = timeStep
    self.localObserver = KalmanFilter( 3, 3 )
    self.localObserver.SetMeasurement( 0, 0, 1.0 )
    self.localObserver.SetMeasurement( 1, 1, 1.0 )
    self.localObserver.SetMeasurement( 2, 2, 1.0 )
    self.localObserver.SetStatePredictionFactor( 0, 1, self.dt )
    self.localObserver.SetStatePredictionFactor( 0, 2, 0.5 * self.dt**2 )
    self.localObserver.SetStatePredictionFactor( 1, 2, self.dt )
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
    newState, prediction = self.remoteObserver.Predict( self.state )
    self.state, estimatedMeasurement = self.localObserver.Process( measurement )
    self.remoteObserver.Update( measurement, estimatedMeasurement )
    self.remoteObserver.Correct()
    return [ prediction[ 0 ], prediction[ 1 ], prediction[ 2 ] ]


