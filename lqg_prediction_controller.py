from lqr_control import GetLQRController 
from kalman_filter import KalmanFilter 

class LQGPredController:
  measurement = [ 0.0, 0.0, 0.0 ]
  outputForce = 0.0
  
  def __init__( self, inertia, damping, stiffness, timeStep ):
    self.dt = timeStep
    self.localObserver = KalmanFilter( 3, 3, 1 )
    self.localObserver.SetMeasurement( 0, 0, 1.0 )
    self.localObserver.SetMeasurement( 1, 1, 1.0 )
    self.localObserver.SetMeasurement( 2, 2, 1.0 )
    self.localObserver.SetStatePredictionFactor( 0, 1, self.dt )
    self.localObserver.SetStatePredictionFactor( 0, 2, 0.5 * self.dt**2 )
    self.localObserver.SetStatePredictionFactor( 1, 2, self.dt )
    self.localObserver.SetStatePredictionFactor( 2, 2, 0.0 )
    self.SetSystem( inertia, damping, stiffness )
    self.predictor = KalmanFilter( 3, 3 )
    self.predictor.SetMeasurement( 0, 0, 1.0 )
    self.predictor.SetMeasurement( 1, 1, 1.0 )
    self.predictor.SetMeasurement( 2, 2, 1.0 )
    #self.predictor.SetStatePredictionFactor( 0, 1, self.dt )
    #self.predictor.SetStatePredictionFactor( 0, 2, 0.5 * self.dt**2 )
    #self.predictor.SetStatePredictionFactor( 1, 2, self.dt )
    self.remoteObserver = KalmanFilter( 3, 3 )
    self.remoteObserver.SetMeasurement( 0, 0, 2.0 )
    self.remoteObserver.SetMeasurement( 1, 1, 2.0 )
    self.remoteObserver.SetMeasurement( 2, 2, 2.0 )
    #self.remoteObserver.SetStatePredictionFactor( 0, 1, self.dt )
    #self.remoteObserver.SetStatePredictionFactor( 0, 2, 0.5 * self.dt**2 )
    #self.remoteObserver.SetStatePredictionFactor( 1, 2, self.dt )
  
  def SetSystem( self, inertia, damping, stiffness ):
    A = [ [ 1, self.dt, 0.5 * self.dt**2 ], [ 0, 1, self.dt ], [ -stiffness / inertia, -damping / inertia, 0 ] ]
    B = [ [ 0 ], [ 0 ], [ 1 / inertia ] ]
    C = [ [ 1, 0, 0 ], [ 0, 1, 0 ], [ 0, 0, 1 ] ]
    self.feedbackGain = GetLQRController( A, B, C, 0.0001 )
    self.localObserver.SetStatePredictionFactor( 2, 0, -stiffness / inertia )
    self.localObserver.SetStatePredictionFactor( 2, 1, -damping / inertia )
    self.localObserver.SetInputPredictionFactor( 2, 0, 1 / inertia )
  
  def Predict( self, remotePacket, timeDelay ):
    timeDelay = int( timeDelay / self.dt ) * self.dt
    remoteMeasurement = [ remotePacket[ 0 ], remotePacket[ 1 ], remotePacket[ 2 ] ]
    self.predictor.SetObservationFactor( 0, 1, timeDelay )
    self.predictor.SetObservationFactor( 0, 2, 0.5 * timeDelay**2 )
    self.predictor.SetObservationFactor( 1, 2, timeDelay )
    remoteState, self.measurement = self.predictor.Predict( remoteMeasurement )
    self.predictor.Update( remoteMeasurement, remoteState )
    remoteState, self.measurement = self.predictor.Correct()
    remoteState, self.measurement = self.remoteObserver.Process( self.measurement )
    return ( self.measurement[ 0 ], self.measurement[ 1 ], self.measurement[ 2 ] )

  def Process( self, localPacket, inputForce ):
    reference = [ localPacket[ 0 ] - self.measurement[ 0 ], 
                 localPacket[ 1 ] - self.measurement[ 1 ], 
                 localPacket[ 2 ] - self.measurement[ 2 ] ]
    referenceState, error = self.localObserver.Process( reference, [ inputForce ] )
    self.outputForce = -self.feedbackGain.dot( referenceState )[ 0 ]
    return self.outputForce
