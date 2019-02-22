from lqg_control import GetLQGController 
from kalman_filter import KalmanFilter 

class LQGController:
  Kp = 10.0
  Kv = 1.0
  setpoint = 0.0
  setpointVelocity = 0.0
  lastPosition = 0.0
  lastVelocity = 0.0
  
  def PreProcess( self, inputPacket, timeDelay, timeDelta ):
    setpoint, setpointVelocity = inputPacket
    A = [ [ 1, timeDelta, timeDelta**2 / 2 ], [ 0, 1, timeDelta ], [ 0, 0, 1 ] ]
    B = [ [ 0 ], [ 0 ], [ -1 / 1.0 ] ]
    C = [ [ 1, 0, 0 ] ]
    self.feedbackGain = GetLQGController( A, B, C, 0.1 )
    self.setpoint = setpoint + ( setpointVelocity + self.setpointVelocity ) * timeDelta / 2
    self.setpointVelocity = setpointVelocity

  def Process( self, inputPosition, inputVelocity, inputForce ):
    self.lastPosition = inputPosition
    self.lastVelocity = inputVelocity
    outputForce = self.Kp * ( self.setpoint - self.lastPosition ) + self.Kv * ( self.setpointVelocity - self.lastVelocity )
    return outputForce

  def PostProcess( self ):
    return ( self.lastPosition, self.lastVelocity )
