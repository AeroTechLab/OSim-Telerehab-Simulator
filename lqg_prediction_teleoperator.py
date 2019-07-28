from lqg_controller import LQGController
from lqg_prediction_controller import LQGPredController
#from lqg_fusion_controller import LQGPredController

class LQGPredTeleoperator:
  
  def __init__( self, inertia, damping, stiffness, timeStep ):
    self.remoteController = LQGController( inertia, damping, stiffness, timeStep )
    self.localController = LQGPredController( inertia, damping, stiffness, timeStep )
  
  def Process( self, localState, remoteState, remoteForce, timeDelay ):
    
    feedforwardForce = self.remoteController.Process( localState, remoteState, remoteForce )
      
    slavePredictedOutput = self.localController.PreProcess( remoteState, timeDelay )
    feedbackForce = self.localController.Process( localState, feedforwardForce + remoteForce )
    slaveCorrectedOutput = self.localController.PostProcess()
    
    return ( feedbackForce, slavePredictedOutput, slaveCorrectedOutput )
