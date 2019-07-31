from wave_position_controller import WaveController

class WaveTeleoperator:
  
  def __init__( self, impedance, timeStep ):
    self.waveController = WaveController( impedance[ 0 ] + impedance[ 1 ] + impedance[ 2 ], timeStep )
    self.localImpedance = impedance
    self.remoteImpedance = impedance
  
  def _SetWaveImpedance( self, remoteImpedance, localImpedance ):
    waveImpedance = ( max( remoteImpedance[ 0 ] + localImpedance[ 0 ], 0.0 ),
                      max( remoteImpedance[ 1 ] + localImpedance[ 1 ], 1.0 ),
                      max( remoteImpedance[ 2 ] + localImpedance[ 2 ], 0.0 ) )
    self.waveController.SetImpedance( waveImpedance[ 0 ] + waveImpedance[ 1 ] + waveImpedance[ 2 ] )
  
  def SetRemoteSystem( self, impedance ):
    self.remoteImpedance = impedance
    self._SetWaveImpedance( self.remoteImpedance, self.localImpedance )
    
  def SetLocalSystem( self, impedance ):
    self.localImpedance = impedance
    self._SetWaveImpedance( self.remoteImpedance, self.localImpedance )
  
  def Process( self, localState, remoteState, externalForce, timeDelay ):
      
    slaveFilteredOutput = self.waveController.PreProcess( remoteState, timeDelay )
    feedbackForce = self.waveController.Process( localState )
    slaveCorrectedOutput = self.waveController.PostProcess()
    
    return ( feedbackForce, slaveCorrectedOutput )
