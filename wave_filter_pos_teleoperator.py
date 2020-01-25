from wave_controller import WaveController

import math

class WaveTeleoperator:
  
  lastInputWave = 0.0
  lastFilteredWave = 0.0
  lastInputIntegral = 0.0
  lastFilteredIntegral = 0.0
  
  def __init__( self, impedance, timeStep ):
    self.waveController = WaveController( impedance[ 0 ] + impedance[ 1 ] + impedance[ 2 ], timeStep )
  
  def SetSystem( self, impedance ):
    waveImpedance = ( max( impedance[ 0 ], 0.0 ), max( impedance[ 1 ], 5.0 ), max( impedance[ 2 ], 0.0 ) )
    self.waveController.SetImpedance( waveImpedance[ 0 ] + waveImpedance[ 1 ] + waveImpedance[ 2 ] )
  
  def Process( self, localState, localForce, remotePacket, timeDelay ):
    
    localPosition, localVelocity, localAcceleration = localState
    inputWave, remotePosition, inputEnergy, remoteImpedance = remotePacket
    
    bandwidth = 0.5
    self.lastFilteredWave = ( ( 2 - bandwidth ) * self.lastFilteredWave + bandwidth * ( inputWave + self.lastInputWave ) ) / ( 2 + bandwidth )
    self.lastInputWave = inputWave
    
    positionError = localPosition - remotePosition
    waveCorrection = -math.sqrt( 2.0 * remoteImpedance ) * bandwidth * positionError
    if positionError * self.lastFilteredWave < 0: waveCorrection = 0
    elif abs( waveCorrection ) > abs( self.lastFilteredWave ): waveCorrection = -self.lastFilteredWave
    self.lastFilteredWave += waveCorrection
    
    remoteVelocity = self.waveController.PreProcess( self.lastFilteredWave, inputEnergy, remoteImpedance )
    feedbackForce, outputWave, outputEnergy, localImpedance = self.waveController.PostProcess( localVelocity, localForce )
    
    return ( feedbackForce, ( remotePosition, remoteVelocity, 0.0 ), ( outputWave, localPosition, outputEnergy, localImpedance ) )
