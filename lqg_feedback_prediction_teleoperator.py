from lqg_prediction_controller import LQGPredController
from mtdpc_stabilizer import MTDPCStabilizer

class LQGFFBPredTeleoperator:
 
  def __init__( self, impedance, timeStep ):
    self.predController = LQGPredController( impedance[ 0 ], 0.0, 0.0, timeStep )
    self.stabilizer = MTDPCStabilizer( 0.7, timeStep )
    self.plantDamping = impedance[ 1 ]
  
  def SetSystem( self, impedance ):
    self.predController.SetSystem( impedance[ 0 ], impedance[ 1 ], impedance[ 2 ] )
    self.plantDamping = impedance[ 1 ]
  
  def Process( self, localState, localForce, remotePacket, timeDelay ):
    *remoteState, remoteForce = remotePacket
    
    remoteCorrectedState, remoteCorrectedForce = self.predController.Predict( remoteState, remoteForce, timeDelay )
    controlForce = self.predController.Process( remoteCorrectedState, localState, remoteCorrectedForce + localForce )    
    
    feedbackForce = self.stabilizer.Process( localForce, controlForce + remoteCorrectedForce, self.plantDamping, localState[ 1 ] )
    
    return ( feedbackForce, remoteCorrectedState, ( *localState, localForce ) )
