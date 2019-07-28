from lqg_controller import LQGController
from lqg_prediction_controller import LQGPredController

class LQGPredTeleoperator:
  
  def __init__( self, impedance, timeStep ):
    self.remoteController = LQGController( impedance[ 0 ], impedance[ 1 ], impedance[ 2 ], timeStep )
    self.localController = LQGPredController( impedance[ 0 ], impedance[ 1 ], impedance[ 2 ], timeStep )
  
  def SetRemoteSystem( self, impedance ):
    self.remoteController.SetSystem( impedance[ 0 ], impedance[ 1 ], impedance[ 2 ] )
    
  def SetLocalSystem( self, impedance ):
    self.localController.SetSystem( impedance[ 0 ], impedance[ 1 ], impedance[ 2 ] )
  
  def Process( self, localState, remoteState, remoteForce, timeDelay ):
    
    feedforwardForce = self.remoteController.Process( localState, remoteState, remoteForce )
      
    remotePredictedState = self.localController.PreProcess( remoteState, timeDelay )
    feedbackForce = self.localController.Process( localState, feedforwardForce + remoteForce )
    remoteCorrectedState = self.localController.PostProcess()
    
    return ( feedbackForce, remotePredictedState, remoteCorrectedState )
