import math

class WaveController:
  waveImpedance = 1.0
  lastInputWave = 0.0
  lastFilteredWave = 0.0
  lastInputIntegral = 0.0
  lastFilteredIntegral = 0.0
  lastVelocity = 0.0
  lastPosition = 0.0
  lastForce = 0.0
  lastMomentum = 0.0
  
  def __init__( self, timeDelta ):
    self.dt = timeDelta
  
  def PreProcess( self, inputPacket, timeDelay ):
    delta = self.dt / timeDelay
    inputWave, inputWaveIntegral = inputPacket
    self.lastFilteredWave = ( ( 2 - delta ) * self.lastFilteredWave + delta * ( inputWave + self.lastInputWave ) ) / ( 2 + delta )
    self.lastInputWave = inputWave
    self.lastFilteredIntegral = ( ( 2 - delta ) * self.lastFilteredIntegral + delta * ( inputWaveIntegral + self.lastInputIntegral ) ) / ( 2 + delta )
    self.lastInputIntegral = inputWaveIntegral

  def Process( self, inputPosition, inputVelocity, inputForce ):
    self.lastForce = - ( self.waveImpedance * inputVelocity - math.sqrt( 2.0 * self.waveImpedance ) * self.lastFilteredWave )
    self.lastMomentum = - ( self.waveImpedance * inputPosition - math.sqrt( 2.0 * self.waveImpedance ) * self.lastFilteredIntegral )
    self.lastVelocity = inputVelocity
    self.lastPosition = inputPosition
    return self.lastForce

  def PostProcess( self ):
    outputWave = ( self.waveImpedance * self.lastVelocity - self.lastForce ) / math.sqrt( 2.0 * self.waveImpedance )  
    outputWaveIntegral = ( self.waveImpedance * self.lastPosition - self.lastMomentum ) / math.sqrt( 2.0 * self.waveImpedance )
    return ( outputWave, outputWaveIntegral )
    
