from lqg_controller import LQGController
from lqg_prediction_controller import LQGPredController

class LQGFFBPredTeleoperator:
 
  def __init__( self, impedance, timeStep ):
    self.localController = LQGPredController( impedance[ 0 ], impedance[ 1 ], impedance[ 2 ], timeStep )
  
  def SetSystem( self, impedance ):
    self.localController.SetSystem( impedance[ 0 ], impedance[ 1 ], impedance[ 2 ] )
  
  def Process( self, localState, localForce, remotePacket, timeDelay ):
    *remoteState, remoteForce = remotePacket
    
    remoteCorrectedState, remoteCorrectedForce = self.localController.Predict( remoteState, remoteForce, timeDelay )
    controlForce = self.localController.Process( remoteCorrectedState, localState, remoteCorrectedForce + localForce )    
    
    return ( controlForce + remoteCorrectedForce, remoteCorrectedState, ( *localState, localForce ) )
