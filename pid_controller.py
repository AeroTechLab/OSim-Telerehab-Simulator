class PIDController:
  Kp = [ 10.0, 0.0, 0.0 ]
  Kv = 0.0
  setpoint = 0.0
  setpointVelocity = 0.0
  lastPosition = 0.0
  lastVelocity = 0.0
  error = [ 0.0, 0.0, 0.0 ]
  output = [ 0.0, 0.0, 0.0 ]
  
  def __init__( self, timeDelay ):
    self.dt = timeDelay
    Kp = 10.0; Kd = 1.0; Ki = 0.1
    self.Kp[ 0 ] = Kp + 2 * Kd / self.dt + self.dt * Ki / 2
    self.Kp[ 1 ] = -4 * Kd / self.dt + self.dt * Ki
    self.Kp[ 2 ] = -Kp + 2 * Kd / self.dt + self.dt * Ki / 2
  
  def PreProcess( self, inputPacket, timeDelay ):
    setpoint, setpointVelocity = inputPacket
    self.setpoint = setpoint + ( setpointVelocity + self.setpointVelocity ) * self.dt / 2
    self.setpointVelocity = setpointVelocity

  def Process( self, inputPosition, inputVelocity, inputForce ):
    self.lastPosition = inputPosition
    self.lastVelocity = inputVelocity
    self.error[ 0 ] = self.setpoint - self.lastPosition
    self.output[ 0 ] = self.output[ 2 ]
    self.output[ 0 ] += self.Kp[ 0 ] * self.error[ 0 ] + self.Kp[ 1 ] * self.error[ 1 ] + self.Kp[ 2 ] * self.error[ 2 ]
    #self.output[ 0 ] += self.Kv * ( self.setpointVelocity - self.lastVelocity )
    return self.output[ 0 ]

  def PostProcess( self ):
    self.error[ 2 ] = self.error[ 1 ]
    self.error[ 1 ] = self.error[ 0 ]
    self.output[ 2 ] = self.output[ 1 ]
    self.output[ 1 ] = self.output[ 0 ]
    return ( self.lastPosition, self.lastVelocity )
