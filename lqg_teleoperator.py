from lqg_controller import LQGController

class LQGTeleoperator:
  feedbackForce = 0.0
  
  def __init__( self, impedance, timeStep ):
    self.controller = LQGController( impedance[ 0 ], impedance[ 1 ], impedance[ 2 ], timeStep )
    self.localImpedance = impedance
    self.remoteImpedance = impedance
    self.passiveImpedance = impedance
  
  def SetRemoteSystem( self, impedance ):
    self.remoteImpedance = impedance
    
  def SetLocalSystem( self, impedance ):
    self.controller.SetSystem( impedance[ 0 ], impedance[ 1 ], impedance[ 2 ] )
    self.localImpedance = impedance
  
  def Process( self, localState, remoteState, externalForce, timeDelay ):
    targetForce = self.localImpedance[ 0 ] * remoteState[ 2 ] + self.localImpedance[ 1 ] * remoteState[ 1 ] + self.localImpedance[ 2 ] * remoteState[ 0 ]
    reactionForce = targetForce #- externalForce
    #targetForce = self.passiveImpedance[ 0 ] * remoteState[ 2 ] + self.passiveImpedance[ 1 ] * remoteState[ 1 ] + self.passiveImpedance[ 2 ] * remoteState[ 0 ]
    #reactionForce = targetForce - externalForce
    reactionForce = -self.remoteImpedance[ 0 ] * localState[ 2 ] - self.remoteImpedance[ 1 ] * localState[ 1 ] - self.remoteImpedance[ 2 ] * localState[ 0 ]
    
    controlForce = self.controller.Process( remoteState, localState, self.feedbackForce )
    
    mixingFactor = 1.0
    self.feedbackForce = mixingFactor * controlForce + ( 1 - mixingFactor ) * reactionForce
    
    return ( self.feedbackForce, remoteState )
