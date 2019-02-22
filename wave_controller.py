import math

class WaveController:
  waveImpedance = 1.0
  filterStrength = 1.0
  lastInputWave = 0.0
  lastFilteredWave = 0.0
  lastInputWaveIntegral = 0.0
  lastFilteredWaveIntegral = 0.0
  lastVelocity = 0.0
  lastPosition = 0.0
  lastForce = 0.0
  lastMomentum = 0.0
  
  def PreProcess( self, inputPacket, timeDelay, timeDelta ):
    relativeTimeDelta = timeDelta / timeDelay
    inputWave, inputWaveIntegral = inputPacket
    self.lastFilteredWave = ( ( 2 - relativeTimeDelta ) * self.lastFilteredWave + relativeTimeDelta * ( inputWave + self.lastInputWave ) ) / ( 2 + relativeTimeDelta )
    self.lastInputWave = inputWave
    self.lastFilteredWaveIntegral = ( ( 2 - relativeTimeDelta ) * self.lastFilteredWaveIntegral + relativeTimeDelta * ( inputWaveIntegral + self.lastInputWaveIntegral ) ) / ( 2 + relativeTimeDelta )
    self.lastInputWaveIntegral = inputWaveIntegral

  def Process( self, inputPosition, inputVelocity, inputForce ):
    self.lastForce = - ( self.waveImpedance * inputVelocity - math.sqrt( 2.0 * self.waveImpedance ) * self.lastFilteredWave )
    self.lastMomentum = - ( self.waveImpedance * inputPosition - math.sqrt( 2.0 * self.waveImpedance ) * self.lastFilteredWaveIntegral )
    self.lastVelocity = inputVelocity
    self.lastPosition = inputPosition
    return self.lastForce

  def PostProcess( self ):
    outputWave = ( self.waveImpedance * self.lastVelocity - self.lastForce ) / math.sqrt( 2.0 * self.waveImpedance )  
    outputWaveIntegral = ( self.waveImpedance * self.lastPosition - self.lastMomentum ) / math.sqrt( 2.0 * self.waveImpedance )
    return ( outputWave, outputWaveIntegral )

  #def PostProcess( self, inputWave, inputForce, timeDelta ):
    #outputForce = -inputForce
    #self.integratedVelocity += self.lastVelocity * timeDelta / 2.0
    #self.lastVelocity = ( math.sqrt( 2.0 * self.waveImpedance ) * inputWave - outputForce ) / self.waveImpedance
    #self.integratedVelocity += self.lastVelocity * timeDelta / 2.0;
    #outputWave = ( self.waveImpedance * self.lastVelocity - outputForce ) / math.sqrt( 2.0 * self.waveImpedance )
    
