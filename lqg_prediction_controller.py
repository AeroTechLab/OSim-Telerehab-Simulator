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
    self.SetLocalSystem( inertia, damping, stiffness )
    self.smithObserver = KalmanFilter( 3, 3, 1 )
    self.smithObserver.SetMeasurement( 0, 0, 2.0 )
    self.smithObserver.SetMeasurement( 1, 1, 2.0 )
    self.smithObserver.SetMeasurement( 2, 2, 2.0 )
    self.smithObserver.SetStatePredictionFactor( 0, 1, self.dt )
    self.smithObserver.SetStatePredictionFactor( 0, 2, 0.5 * self.dt**2 )
    self.smithObserver.SetStatePredictionFactor( 1, 2, self.dt )
    self.SetRemoteSystem( inertia, damping, stiffness )
    self.remoteObserver = KalmanFilter( 3, 3 )
    self.remoteObserver.SetMeasurement( 0, 0, 1.0 )
    self.remoteObserver.SetMeasurement( 1, 1, 1.0 )
    self.remoteObserver.SetMeasurement( 2, 2, 1.0 )
    self.remoteObserver.SetStatePredictionFactor( 0, 1, self.dt )
    self.remoteObserver.SetStatePredictionFactor( 0, 2, 0.5 * self.dt**2 )
    self.remoteObserver.SetStatePredictionFactor( 1, 2, self.dt )
  
  def SetLocalSystem( self, inertia, damping, stiffness ):
    A = [ [ 1, self.dt, 0.5 * self.dt**2 ], [ 0, 1, self.dt ], [ -stiffness / inertia, -damping / inertia, 0 ] ]
    B = [ [ 0 ], [ 0 ], [ 1 / inertia ] ]
    C = [ [ 1, 0, 0 ], [ 0, 1, 0 ], [ 0, 0, 1 ] ]
    self.feedbackGain = GetLQRController( A, B, C, 0.0001 )
    self.localObserver.SetStatePredictionFactor( 2, 0, -stiffness / inertia )
    self.localObserver.SetStatePredictionFactor( 2, 1, -damping / inertia )
    self.localObserver.SetInputPredictionFactor( 2, 0, 1 / inertia )
  
  def SetRemoteSystem( self, inertia, damping, stiffness ):
    self.smithObserver.SetStatePredictionFactor( 2, 0, -stiffness / inertia )
    self.smithObserver.SetStatePredictionFactor( 2, 1, -damping / inertia )
    self.smithObserver.SetInputPredictionFactor( 2, 0, 1 / inertia )
  
  def PreProcess( self, remotePacket, timeDelay ):
    timeDelay = int( timeDelay / self.dt ) * self.dt
    remoteMeasurement = [ remotePacket[ 0 ], remotePacket[ 1 ], remotePacket[ 2 ] ]
    self.remoteObserver.SetObservationFactor( 0, 1, timeDelay )
    self.remoteObserver.SetObservationFactor( 0, 2, 0.5 * timeDelay**2 )
    self.remoteObserver.SetObservationFactor( 1, 2, timeDelay )
    remoteState, self.measurement = self.remoteObserver.Predict( remoteMeasurement )
    self.remoteObserver.Update( remoteMeasurement, remoteState )
    remoteState, self.measurement = self.remoteObserver.Correct()
    return ( self.measurement[ 0 ], self.measurement[ 1 ], self.measurement[ 2 ] )

  def Process( self, localPacket, remoteForce, externalForce ):
    remoteState, self.measurement = self.smithObserver.Process( self.measurement, [ remoteForce ] )
    reference = [ localPacket[ 0 ] - remoteState[ 0 ], localPacket[ 1 ] - remoteState[ 1 ], localPacket[ 2 ] - remoteState[ 2 ] ]
    referenceState, error = self.localObserver.Process( reference, [ externalForce + self.outputForce ] )
    self.outputForce = -self.feedbackGain.dot( referenceState )[ 0 ]
    return self.outputForce

  def PostProcess( self ):
    return ( self.measurement[ 0 ], self.measurement[ 1 ], self.measurement[ 2 ] )
