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
  bandwidth = 0.5
  
  def __init__( self, impedance, timeStep ):
    self.SetImpedance( impedance )
    self.dt = timeStep
  
  def SetImpedance( self, impedance ):
    self.waveImpedance = max( impedance, 1.0 )
  
  def PreProcess( self, remotePacket, timeDelay ):
    delta = self.bandwidth
    remotePosition, inputWave, dummy = remotePacket
    self.lastFilteredWave = ( ( 2 - delta ) * self.lastFilteredWave + delta * ( inputWave + self.lastInputWave ) ) / ( 2 + delta )
    self.lastInputWave = inputWave
    self.lastRemotePosition = remotePosition
    self.bandwidth = delta
    return ( self.lastRemotePosition, self.lastFilteredWave, 0.0 )

  def Process( self, localPacket ):
    positionError = localPacket[ 0 ] - self.lastRemotePosition
    waveCorrection = -math.sqrt( 2.0 * self.waveImpedance ) * self.bandwidth * positionError
    if positionError * self.lastFilteredWave < 0: waveCorrection = 0
    elif abs( waveCorrection ) > abs( self.lastFilteredWave ): waveCorrection = -self.lastFilteredWave
    self.lastFilteredWave += waveCorrection
    self.lastForce = -( self.waveImpedance * localPacket[ 1 ] - math.sqrt( 2.0 * self.waveImpedance ) * self.lastFilteredWave )
    self.lastVelocity = localPacket[ 1 ]
    self.lastPosition = localPacket[ 0 ]
    return self.lastForce

  def PostProcess( self, localForce ):
    outputWave = ( self.waveImpedance * self.lastVelocity - self.lastForce ) / math.sqrt( 2.0 * self.waveImpedance )  
    #outputWave = ( self.waveImpedance * self.lastVelocity + localForce ) / math.sqrt( 2.0 * self.waveImpedance )  
    return ( self.lastRemotePosition, outputWave )
