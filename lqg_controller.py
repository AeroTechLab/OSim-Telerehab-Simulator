from lqg_control import GetLQGController 
from kalman_filter import KalmanFilter 

class LQGController:
  Kp = 10.0
  Kv = 1.0
  setpoint = 0.0
  setpointVelocity = 0.0
  lastPosition = 0.0
  lastVelocity = 0.0
  
  def __init__( self, timeDelta ):
    self.dt = timeDelta
    A = [ [ 1, self.dt, self.dt**2 / 2 ], [ 0, 1, self.dt ], [ 0, 0, 1 ] ]
    B = [ [ 0 ], [ 0 ], [ -1 / 1.0 ] ]
    C = [ [ 1, 0, 0 ], [ 0, 1, 0 ], [ 0, 0, 1 ] ]
    self.feedbackGain = GetLQGController( A, B, C, 0.1 )
    self.remoteObserver = KalmanFilter( 3, 3 )
    self.remoteObserver.SetMeasurement( 0, 0, 0.1 )
    self.remoteObserver.SetMeasurement( 1, 1, 0.1 )
    self.remoteObserver.SetMeasurement( 2, 2, 0.1 )
  
  def PreProcess( self, inputPacket, timeDelay ):
    position, velocity, acceleration = inputPacket
    self.remoteObserver.SetStatePredictionFactor( 0, 1, timeDelay )
    self.remoteObserver.SetStatePredictionFactor( 0, 2, ( timeDelay**2 ) / 2 )
    self.remoteObserver.SetStatePredictionFactor( 1, 2, timeDelay )
    state, measurement = self.remoteObserver.Update( [ position, velocity, acceleration ] )
    outputForce = -self.feedbackGain.dot( state )[ 0 ]
    self.setpoint = setpoint + ( setpointVelocity + self.setpointVelocity ) * self.dt / 2
    self.setpointVelocity = setpointVelocity

  def Process( self, inputPosition, inputVelocity, inputForce ):
    self.lastPosition = inputPosition
    self.lastVelocity = inputVelocity
    outputForce = self.Kp * ( self.setpoint - self.lastPosition ) + self.Kv * ( self.setpointVelocity - self.lastVelocity )
    return outputForce

  def PostProcess( self ):
    return ( self.lastPosition, self.lastVelocity )
