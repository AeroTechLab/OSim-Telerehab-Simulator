from lqg_controller import LQGController
from lqg_prediction_controller import LQGPredController

class LQGPredTeleoperator:
  feedbackForce = 0.0
 
  def __init__( self, impedance, timeStep ):
    self.localController = LQGPredController( impedance[ 0 ], impedance[ 1 ], impedance[ 2 ], timeStep )
    self.localImpedance = impedance
  
  def SetRemoteSystem( self, impedance ):
    pass
    
  def SetLocalSystem( self, impedance ):
    self.localController.SetSystem( impedance[ 0 ], impedance[ 1 ], impedance[ 2 ] )
    self.localImpedance = impedance
  
  def Process( self, localState, remoteState, externalForce, timeDelay ):         
    remoteCorrectedState = self.localController.Predict( remoteState, timeDelay )
    
    targetForce = self.localImpedance[ 0 ] * remoteCorrectedState[ 2 ] + self.localImpedance[ 1 ] * remoteCorrectedState[ 1 ] + self.localImpedance[ 2 ] * remoteCorrectedState[ 0 ]
    reactionForce = targetForce - externalForce
    
    controlForce = self.localController.Process( localState, self.feedbackForce )    
    
    mixingFactor = 1.0
    self.feedbackForce = mixingFactor * controlForce + ( 1 - mixingFactor ) * reactionForce
    
    return ( self.feedbackForce, remoteCorrectedState )
