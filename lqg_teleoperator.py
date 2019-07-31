from lqg_controller import LQGController

class LQGTeleoperator:
  
  def __init__( self, impedance, timeStep ):
    self.controller = LQGController( impedance[ 0 ], impedance[ 1 ], impedance[ 2 ], timeStep )
  
  def SetRemoteSystem( self, impedance ):
    pass
    
  def SetLocalSystem( self, impedance ):
    self.controller.SetSystem( impedance[ 0 ], impedance[ 1 ], impedance[ 2 ] )
  
  def Process( self, localState, remoteState, localForce, remoteForce, timeDelay ):
    controlForce = self.controller.Process( remoteState, localState, remoteForce + localForce )
    
    return ( controlForce + remoteForce, remoteState )
