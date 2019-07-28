from pid_controller import PIDController

class PIDTeleoperator:
  
  def __init__( self, impedance, timeStep ):
    self.controller = PIDController( timeStep )
  
  def SetRemoteSystem( self, impedance ):
    pass
    
  def SetLocalSystem( self, impedance ):
    pass
  
  def Process( self, localState, remoteState, remoteForce, timeDelay ):      
    self.controller.PreProcess( remoteState, timeDelay )
    feedbackForce = self.controller.Process( localState, remoteForce )
    remotePredictedState = self.controller.PostProcess()
    
    return ( feedbackForce, remotePredictedState, remotePredictedState )
