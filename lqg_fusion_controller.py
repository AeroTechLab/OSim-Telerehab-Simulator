from lqr_control import GetLQRController 
from kalman_filter import KalmanFilter 

class LQGController:

  def __init__( self, inertia, damping, stiffness, timeStep ):
    A = [ [ 1, timeStep, 0.5 * timeStep**2 ], [ 0, 1, timeStep ], [ -stiffness / inertia, -damping / inertia, 0 ] ]
    B = [ [ 0 ], [ 0 ], [ 1 / inertia ] ]
    C = [ [ 1, 0, 0 ], [ 0, 1, 0 ], [ 0, 0, 1 ] ]
    self.feedbackGain = GetLQRController( A, B, C, 0.0001 )
    self.observer = KalmanFilter( 3, 2 )
    self.observer.SetMeasurement( 0, 0, 1.0 )
    self.observer.SetMeasurement( 1, 1, 1.0 )
    self.observer.SetStatePredictionFactor( 0, 1, A[ 0 ][ 1 ] )
    self.observer.SetStatePredictionFactor( 0, 2, A[ 0 ][ 2 ] )
    self.observer.SetStatePredictionFactor( 1, 2, A[ 1 ][ 2 ] )
    self.observer.SetStatePredictionFactor( 2, 2, A[ 2 ][ 2 ] )
    self.observer.SetInputPredictionFactor( 2, 0, B[ 2 ][ 0 ] )

  def Process( self, measurePacket, setpointPacket ):
    position, velocity, inputForce = measurePacket
    setpoint, setpointVelocity, feedbackForce = setpointPacket
    reference = [ position - setpoint, velocity - setpointVelocity ]
    state, measurement = self.observer.Process( reference, [ inputForce + feedbackForce ] )
    controlForce = -self.feedbackGain.dot( state )[ 0 ]
    return ( controlForce, [ state[ 0 ], state[ 1 ], state[ 2 ] ] )

