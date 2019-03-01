from lqg_control import GetLQGController 
from kalman_filter import KalmanFilter 

class LQGController:
  state = [ 0.0, 0.0, 0.0 ]
  reference = [ 0.0, 0.0 ]
  setpoint = 0.0
  setpointVelocity = 0.0
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
  
  def PreProcess( self, inputPacket, timeDelay ):
    self.setpoint, self.setpointVelocity, dummy = inputPacket

  def Process( self, inputPosition, inputVelocity, inputForce ):
    self.reference[ 0 ] = inputPosition - self.setpoint
    self.reference[ 1 ] = inputVelocity - self.setpointVelocity
    self.state, measurement = self.localObserver.Process( self.reference, [ inputForce + self.outputForce ] )
    self.outputForce = -self.feedbackGain.dot( self.state )[ 0 ]
    return self.outputForce

  def PostProcess( self ):
    return ( self.state[ 0 ], self.state[ 1 ], self.state[ 2 ] )
