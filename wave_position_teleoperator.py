from wave_position_controller import WaveController

class WaveTeleoperator:
  
  def __init__( self, inertia, damping, stiffness, timeStep ):
    self.waveController = WaveController( 1.0, timeStep )
  
  def Process( self, localState, remoteState, remoteForce, timeDelay ):
      
    slavePredictedOutput = self.waveController.PreProcess( remoteState, timeDelay )
    feedbackForce = self.waveController.Process( localState, remoteForce )
    slaveCorrectedOutput = self.waveController.PostProcess()
    
    return ( feedbackForce, slavePredictedOutput, slaveCorrectedOutput )
