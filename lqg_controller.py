from lqr_control import GetLQRController 
from kalman_filter import KalmanFilter 

class LQGController:
  controlForce = 0.0
  
  def __init__( self, inertia, damping, stiffness, timeStep ):
    self.dt = timeStep
    A = [ [ 1, self.dt, 0.5 * self.dt**2 ], [ 0, 1, self.dt ], [ -stiffness / inertia, -damping / inertia, 0 ] ]
    B = [ [ 0 ], [ 0 ], [ 1 / inertia ] ]
    C = [ [ 1, 0, 0 ], [ 0, 1, 0 ], [ 0, 0, 1 ] ]
    self.feedbackGain = GetLQRController( A, B, C, 0.0001 )
    self.observer = KalmanFilter( 3, 3, 1 )
    self.observer.SetMeasurement( 0, 0, 1.0 )
    self.observer.SetMeasurement( 1, 1, 1.0 )
    self.observer.SetMeasurement( 2, 2, 1.0 )
    self.observer.SetStatePredictionFactor( 0, 1, self.dt )
    self.observer.SetStatePredictionFactor( 0, 2, 0.5 * self.dt**2 )
    self.observer.SetStatePredictionFactor( 1, 2, self.dt )
    self.observer.SetStatePredictionFactor( 2, 2, 0.0 )
    self.SetSystem( inertia, damping, stiffness )
    
  def SetSystem( self, inertia, damping, stiffness ):
    self.observer.SetStatePredictionFactor( 2, 0, -stiffness / inertia )
    self.observer.SetStatePredictionFactor( 2, 1, -damping / inertia )
    self.observer.SetInputPredictionFactor( 2, 0, 1 / inertia )
    
  def Process( self, setpoint, measurement, externalForce ):
    reference = [ measurement[ 0 ] - setpoint[ 0 ], measurement[ 1 ] - setpoint[ 1 ], measurement[ 2 ] - setpoint[ 2 ] ]
    state, error = self.observer.Process( reference, [ self.controlForce + externalForce ] )
    self.controlForce = -self.feedbackGain.dot( state )[ 0 ]
    return self.controlForce
