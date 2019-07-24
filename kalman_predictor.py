from kalman_filter import KalmanFilter 

class KalmanPredictor:
  state = [ 0.0, 0.0, 0.0 ]
  predictions = []
  
  def __init__( self, timeDelta ):
    self.dt = timeDelta
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
    #self.state = [ setpoint, setpointVelocity, setpointAcceleration ]
    self.state, local = self.localObserver.Predict( self.state )
    measurement = [ setpoint, setpointVelocity, setpointAcceleration ]
    timeDelay = int( timeDelay / self.dt ) * self.dt
    self.remoteObserver.SetStatePredictionFactor( 0, 1, timeDelay )
    self.remoteObserver.SetStatePredictionFactor( 0, 2, 0.5 * timeDelay**2 )
    self.remoteObserver.SetStatePredictionFactor( 1, 2, timeDelay )
    self.state, prediction = self.remoteObserver.Predict( measurement )
    if len( self.predictions ) > 0:
      if remoteTime >= self.predictions[ 0 ][ 1 ]:
        estimatedMeasurement = self.predictions.pop( 0 )[ 0 ]
        self.remoteObserver.Update( measurement, estimatedMeasurement )
        self.remoteObserver.Correct()
    self.predictions.append( ( prediction, remoteTime + timeDelay ) )
    return ( 0.5 * prediction[ 0 ] + 0.5 * local[ 0 ], prediction[ 1 ], prediction[ 2 ] )
  
  #def Smooth( self ):
    #self.state, measurement = self.remoteObserver.Smooth()
    #return ( measurement[ 0 ], measurement[ 1 ], measurement[ 2 ] )

