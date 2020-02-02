from lqg_controller import LQGController
from mtdpc_stabilizer import MTDPCStabilizer

class LQGFFBTeleoperator:
  remoteCorrectedForce = 0.0
  
  def __init__( self, impedance, timeStep ):
    self.controller = LQGController( impedance[ 0 ], 0.0, 0.0, timeStep )
    self.stabilizer = MTDPCStabilizer( 0.7, timeStep )
    self.plantDamping = impedance[ 1 ]
  
  def SetSystem( self, impedance ):
    self.controller.SetSystem( impedance[ 0 ], impedance[ 1 ], impedance[ 2 ] )
    self.plantDamping = impedance[ 1 ]
  
  def Process( self, localState, localForce, remotePacket, timeDelay ):
    *remoteState, remoteForce = remotePacket
    
    controlForce = self.controller.Process( remoteState, localState, self.remoteCorrectedForce + localForce )
    
    feedbackForce = self.stabilizer.Process( controlForce + remoteForce, self.plantDamping, localState[ 1 ] )
    self.remoteCorrectedForce = feedbackForce - controlForce
    
    return ( feedbackForce, remoteState, ( *localState, localForce ) )
