from pv_controller import PVController

class PVTeleoperator:
  
  def __init__( self, impedance, timeStep ):
    self.controller = PVController( timeStep )
  
  def SetSystem( self, impedance ):
    pass
  
  def Process( self, localState, localForce, remotePacket, timeDelay ):   
    *remoteState, remoteForce = remotePacket
    
    controForce = self.controller.Process( remoteState, localState )
    
    return ( controForce, remoteState, ( *localState, localForce ) )
