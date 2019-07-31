from lqr_control import GetLQRController 
from kalman_filter import KalmanFilter 

class LQGPredController:
  measurement = [ 0.0, 0.0, 0.0 ]
  controlForce = 0.0
  
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
    self.remoteStateObserver = KalmanFilter( 3, 3 )
    self.remoteStateObserver.SetMeasurement( 0, 0, 1.0 )
    self.remoteStateObserver.SetMeasurement( 1, 1, 1.0 )
    self.remoteStateObserver.SetMeasurement( 2, 2, 1.0 )
    self.remoteInputObserver = KalmanFilter( 3, 1 )
    self.remoteInputObserver.SetMeasurement( 0, 0, 1.0 )
  
  def SetSystem( self, inertia, damping, stiffness ):
    A = [ [ 1, self.dt, 0.5 * self.dt**2 ], [ 0, 1, self.dt ], [ -stiffness / inertia, -damping / inertia, 0 ] ]
    B = [ [ 0 ], [ 0 ], [ 1 / inertia ] ]
    C = [ [ 1, 0, 0 ], [ 0, 1, 0 ], [ 0, 0, 1 ] ]
    self.feedbackGain = GetLQRController( A, B, C, 0.0001 )
    self.localObserver.SetStatePredictionFactor( 2, 0, -stiffness / inertia )
    self.localObserver.SetStatePredictionFactor( 2, 1, -damping / inertia )
    self.localObserver.SetInputPredictionFactor( 2, 0, 1 / inertia )
  
  def Predict( self, remotePacket, remoteForce, timeDelay ):
    timeDelay = int( timeDelay / self.dt ) * self.dt
    remoteMeasurement = [ remotePacket[ 0 ], remotePacket[ 1 ], remotePacket[ 2 ] ]
    self.remoteStateObserver.SetStatePredictionFactor( 0, 1, timeDelay )
    self.remoteStateObserver.SetStatePredictionFactor( 0, 2, 0.5 * timeDelay**2 )
    self.remoteStateObserver.SetStatePredictionFactor( 1, 2, timeDelay )
    state, remoteMeasurement = self.remoteStateObserver.Process( remoteMeasurement )
    self.remoteInputObserver.SetStatePredictionFactor( 0, 1, timeDelay )
    self.remoteInputObserver.SetStatePredictionFactor( 0, 2, 0.5 * timeDelay**2 )
    self.remoteInputObserver.SetStatePredictionFactor( 1, 2, timeDelay )
    state, remoteForce = self.remoteInputObserver.Process( [ remoteForce ] )
    return ( ( remoteMeasurement[ 0 ], remoteMeasurement[ 1 ], remoteMeasurement[ 2 ] ), remoteForce[ 0 ] )

  def Process( self, localPacket, externalForce ):
    reference = [ localPacket[ 0 ] - self.measurement[ 0 ], 
                  localPacket[ 1 ] - self.measurement[ 1 ], 
                  localPacket[ 2 ] - self.measurement[ 2 ] ]
    referenceState, error = self.localObserver.Process( reference, [ self.controlForce + externalForce ] )
    self.controlForce = -self.feedbackGain.dot( referenceState )[ 0 ]
    return self.controlForce
