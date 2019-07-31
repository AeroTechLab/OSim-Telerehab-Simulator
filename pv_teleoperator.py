from pv_controller import PVController

class PVTeleoperator:
  
  def __init__( self, impedance, timeStep ):
    self.controller = PVController( timeStep )
  
  def SetRemoteSystem( self, impedance ):
    pass
    
  def SetLocalSystem( self, impedance ):
    pass
  
  def Process( self, localState, remoteState, externalForce, timeDelay ):      
    feedbackForce = self.controller.Process( remoteState, localState )
    
    return ( feedbackForce, remoteState )
