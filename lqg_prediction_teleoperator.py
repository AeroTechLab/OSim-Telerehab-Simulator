from lqg_controller import LQGController
from lqg_prediction_controller import LQGPredController

class LQGPredTeleoperator:
 
  def __init__( self, impedance, timeStep ):
    self.localController = LQGPredController( impedance[ 0 ], impedance[ 1 ], impedance[ 2 ], timeStep )
  
  def SetRemoteSystem( self, impedance ):
    pass
    
  def SetLocalSystem( self, impedance ):
    self.localController.SetSystem( impedance[ 0 ], impedance[ 1 ], impedance[ 2 ] )
  
  def Process( self, localState, remoteState, localForce, remoteForce, timeDelay ):         
    remoteCorrectedState, remoteCorrectedForce = self.localController.Predict( remoteState, remoteForce, timeDelay )
    controlForce = self.localController.Process( localState, remoteCorrectedForce + localForce )    
    
    return ( controlForce + remoteCorrectedForce, remoteCorrectedState )
