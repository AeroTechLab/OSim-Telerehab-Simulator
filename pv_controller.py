class PVController:
  Kp = 10.0
  Kv = 1.0
  setpoint = 0.0
  setpointVelocity = 0.0
  lastPosition = 0.0
  lastVelocity = 0.0
  
  def __init__( self, timeDelta ):
    self.dt = timeDelta
  
  def Process( self, setpointState, measureState ):
    setpoint, setpointVelocity, setpointAcceleration = setpointState
    self.setpoint = setpoint + ( setpointVelocity + self.setpointVelocity ) * self.dt / 2
    self.setpoint += self.setpointVelocity * self.dt / 2
    self.setpointVelocity = setpointVelocity
    self.lastPosition, self.lastVelocity, acceleration = measureState
    outputForce = self.Kp * ( self.setpoint - self.lastPosition ) + self.Kv * ( self.setpointVelocity - self.lastVelocity )
    return outputForce
