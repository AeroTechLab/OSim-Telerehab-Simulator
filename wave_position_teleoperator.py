from wave_position_controller import WaveController

class WaveTeleoperator:
  
  def __init__( self, impedance, timeStep ):
    self.waveController = WaveController( impedance[ 0 ] + impedance[ 1 ] + impedance[ 2 ], timeStep )
  
  def SetSystem( self, impedance ):
    waveImpedance = ( max( impedance[ 0 ], 0.0 ), max( impedance[ 1 ], 1.0 ), max( impedance[ 2 ], 0.0 ) )
    self.waveController.SetImpedance( waveImpedance[ 0 ] + waveImpedance[ 1 ] + waveImpedance[ 2 ] )
  
  def Process( self, localState, remoteState, localForce, remoteForce, timeDelay ):
      
    slaveFilteredOutput = self.waveController.PreProcess( remoteState, timeDelay )
    feedbackForce = self.waveController.Process( localState )
    slaveCorrectedOutput = self.waveController.PostProcess()
    
    return ( feedbackForce, slaveCorrectedOutput )
