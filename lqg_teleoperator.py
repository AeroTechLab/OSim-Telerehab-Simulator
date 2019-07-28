from lqg_controller import LQGController

class LQGTeleoperator:
  
  def __init__( self, impedance, timeStep ):
    self.controller = LQGController( impedance[ 0 ], impedance[ 1 ], impedance[ 2 ], timeStep )
  
  def SetRemoteSystem( self, impedance ):
    pass
    
  def SetLocalSystem( self, impedance ):
    self.controller.SetSystem( impedance[ 0 ], impedance[ 1 ], impedance[ 2 ] )
  
  def Process( self, localState, remoteState, externalForce, timeDelay ):
    feedbackForce = self.controller.Process( remoteState, localState, externalForce )
    
    return ( feedbackForce, remoteState, remoteState )
