from lqg_controller import LQGController
from mtdpc_stabilizer import MTDPCStabilizer

class LQGTeleoperator:
    
  def __init__( self, impedance, timeStep ):
    self.controller = LQGController( impedance[ 0 ], 0.0, 0.0, timeStep )
    self.stabilizer = MTDPCStabilizer( 0.7, timeStep )
    self.plantDamping = impedance[ 1 ]
  
  def SetSystem( self, impedance ):
    self.controller.SetSystem( impedance[ 0 ], impedance[ 1 ], impedance[ 2 ] )
    self.plantDamping = impedance[ 1 ]
  
  def Process( self, localState, localForce, remotePacket, timeDelay ):
    *remoteState, remoteForce = remotePacket
    
    controlForce = self.controller.Process( remoteState, localState, localForce )
    
    controlForce = self.stabilizer.Process( localForce, controlForce, self.plantDamping, localState[ 1 ] )
    
    return ( controlForce, remoteState, ( *localState, localForce ) )
