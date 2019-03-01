import math

class WaveController:
  waveImpedance = 1.0
  lastInputWave = 0.0
  lastFilteredWave = 0.0
  lastRemotePosition = 0.0
  lastVelocity = 0.0
  lastPosition = 0.0
  lastForce = 0.0
  lastMomentum = 0.0
  bandwidth = 1.0
  
  def __init__( self, timeDelta ):
    self.dt = timeDelta
  
  def PreProcess( self, inputPacket, timeDelay ):
    delta = self.dt / timeDelay
    inputWave, remotePosition, dummy = inputPacket
    self.lastFilteredWave = ( ( 2 - delta ) * self.lastFilteredWave + delta * ( inputWave + self.lastInputWave ) ) / ( 2 + delta )
    self.lastInputWave = inputWave
    self.lastRemotePosition = remotePosition
    self.bandwidth = delta

  def Process( self, inputPosition, inputVelocity, inputForce ):
    positionError = inputPosition - self.lastRemotePosition
    waveCorrection = - math.sqrt( 2.0 * self.waveImpedance ) * self.bandwidth * positionError
    if positionError * self.lastFilteredWave < 0: waveCorrection = 0
    elif abs( waveCorrection ) > abs( self.lastFilteredWave ): waveCorrection = - self.lastFilteredWave
    self.lastFilteredWave += waveCorrection
    self.lastForce = - ( self.waveImpedance * inputVelocity - math.sqrt( 2.0 * self.waveImpedance ) * self.lastFilteredWave )
    self.lastVelocity = inputVelocity
    self.lastPosition = inputPosition
    return self.lastForce

  def PostProcess( self ):
    outputWave = ( self.waveImpedance * self.lastVelocity - self.lastForce ) / math.sqrt( 2.0 * self.waveImpedance )  
    return ( outputWave, self.lastPosition, 0.0 )
    
