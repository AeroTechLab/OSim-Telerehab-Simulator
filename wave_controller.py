import math

class WaveController:
  waveImpedance = 1.0
  localImpedance = 1.0
  lastInputWave = 0.0
  lastVelocity = 0.0
  lastPosition = 0.0
  lastForce = 0.0
  inputEnergy = 0.0
  outputEnergy = 0.0
  
  def __init__( self, localImpedance, timeStep ):
    self.SetImpedance( localImpedance )
    self.dt = timeStep
  
  def SetImpedance( self, localImpedance ):
    self.localImpedance = localImpedance
  
  def PreProcess( self, inputWave, inputEnergy, remoteImpedance ):
    self.inputEnergy += 0.5 * inputWave * inputWave * self.dt
    self.lastInputWave = inputWave if inputEnergy - self.inputEnergy > 0.0 else 0.0
    self.waveImpedance = ( self.waveImpedance + max( self.localImpedance, remoteImpedance ) ) / 2
    remoteVelocity = - ( self.lastForce - math.sqrt( 2.0 * self.waveImpedance ) * self.lastInputWave ) / self.waveImpedance
    return remoteVelocity

  def PostProcess( self, localVelocity, localForce ):
    outputForce = - ( self.waveImpedance * localVelocity - math.sqrt( 2.0 * self.waveImpedance ) * self.lastInputWave )
    outputWave = ( self.waveImpedance * localVelocity - outputForce ) / math.sqrt( 2.0 * self.waveImpedance )  
    self.outputEnergy += 0.5 * outputWave * outputWave * self.dt
    return ( outputForce, outputWave, self.outputEnergy, self.localImpedance )
    
