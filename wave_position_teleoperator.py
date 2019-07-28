from wave_position_controller import WaveController

class WaveTeleoperator:
  
  def __init__( self, impedance, timeStep ):
    self.waveController = WaveController( impedance[ 0 ] + impedance[ 1 ] + impedance[ 2 ], timeStep )
  
  def SetRemoteSystem( self, impedance ):
    self.waveController.SetImpedance( impedance[ 0 ] + impedance[ 1 ] + impedance[ 2 ] )
    
  def SetLocalSystem( self, impedance ):
    pass
  
  def Process( self, localState, remoteState, externalForce, timeDelay ):
      
    slavePredictedOutput = self.waveController.PreProcess( remoteState, timeDelay )
    feedbackForce = self.waveController.Process( localState )
    slaveCorrectedOutput = self.waveController.PostProcess()
    
    return ( feedbackForce, slavePredictedOutput, slaveCorrectedOutput )
