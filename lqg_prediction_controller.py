from lqr_control import GetLQRController 
from kalman_filter import KalmanFilter 

class LQGPredController:
  measurement = [ 0.0, 0.0, 0.0 ]
  outputForce = 0.0
  
  def __init__( self, inertia, damping, stiffness, timeStep ):
    self.dt = timeStep
    self.SetLocalSystem( inertia, damping, stiffness )
    self.smithObserver = KalmanFilter( 3, 3, 1 )
    self.smithObserver.SetMeasurement( 0, 0, 2.0 )
    self.smithObserver.SetMeasurement( 1, 1, 2.0 )
    self.smithObserver.SetMeasurement( 2, 2, 2.0 )
    self.SetLocalSystem( inertia, damping, stiffness )
    self.remoteObserver = KalmanFilter( 3, 3 )
    self.remoteObserver.SetMeasurement( 0, 0, 1.0 )
    self.remoteObserver.SetMeasurement( 1, 1, 1.0 )
    self.remoteObserver.SetMeasurement( 2, 2, 1.0 )
    self.remoteObserver.SetStatePredictionFactor( 0, 1, self.dt )
    self.remoteObserver.SetStatePredictionFactor( 0, 2, 0.5 * self.dt**2 )
    self.remoteObserver.SetStatePredictionFactor( 1, 2, self.dt )
    self.predictor = KalmanFilter( 3, 3 )
    self.predictor.SetMeasurement( 0, 0, 1.0 )
    self.predictor.SetMeasurement( 1, 1, 1.0 )
    self.predictor.SetMeasurement( 2, 2, 1.0 )
  
  def SetLocalSystem( self, inertia, damping, stiffness ):
    A = [ [ 1, self.dt, 0.5 * self.dt**2 ], [ 0, 1, self.dt ], [ -stiffness / inertia, -damping / inertia, 0 ] ]
    B = [ [ 0 ], [ 0 ], [ 1 / inertia ] ]
    C = [ [ 1, 0, 0 ], [ 0, 1, 0 ], [ 0, 0, 1 ] ]
    self.feedbackGain = GetLQRController( A, B, C, 0.0001 )
  
  def SetRemoteSystem( self, inertia, damping, stiffness ):
    self.smithObserver.SetStatePredictionFactor( 2, 0, -stiffness / inertia )
    self.smithObserver.SetStatePredictionFactor( 2, 1, -damping / inertia )
    self.smithObserver.SetInputPredictionFactor( 2, 0, 1 / inertia )
  
  def PreProcess( self, remotePacket, timeDelay ):
    remoteMeasurement = [ remotePacket[ 0 ], remotePacket[ 1 ], remotePacket[ 2 ] ]
    remoteState, self.measurement = self.remoteObserver.Process( remoteMeasurement )
    timeDelay = int( timeDelay / self.dt ) * self.dt
    self.predictor.SetStatePredictionFactor( 0, 1, timeDelay )
    self.predictor.SetStatePredictionFactor( 0, 2, 0.5 * timeDelay**2 )
    self.predictor.SetStatePredictionFactor( 1, 2, timeDelay )
    remoteState, self.measurement = self.predictor.Predict( remoteState )
    return ( self.measurement[ 0 ], self.measurement[ 1 ], self.measurement[ 2 ] )

  def Process( self, localPacket, inputForce ):
    remoteState, self.measurement = self.smithObserver.Process( self.measurement, [ inputForce ] )
    referenceState = [ localPacket[ 0 ] - remoteState[ 0 ], localPacket[ 1 ] - remoteState[ 1 ], localPacket[ 2 ] - remoteState[ 2 ] ]
    self.outputForce = -self.feedbackGain.dot( referenceState )[ 0 ]
    return self.outputForce

  def PostProcess( self ):
    return ( self.measurement[ 0 ], self.measurement[ 1 ], self.measurement[ 2 ] )
