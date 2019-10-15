class MTDPCStabilizer:
  def __init__( self, forgettingFactor, timeStep ):
    self.extraEnergy = 0.0
    self.dt = timeStep
    self.gamma = forgettingFactor
  
  def Process( self, feedbackForce, plantDamping, velocity ):
    feedbackPower = feedbackForce * velocity
    dampingPower = plantDamping * velocity * velocity
    if dampingPower < 0.0: dampingPower = 0.0
    energyDiff = ( feedbackPower - dampingPower ) * self.dt
    self.extraEnergy = self.gamma * self.extraEnergy + energyDiff
    extraDampingForce = 0.0
    if abs( velocity ) > 0.001 and self.extraEnergy > 0.0:
      extraDampingForce = self.extraEnergy / ( velocity * self.dt )
    self.extraEnergy -= extraDampingForce * velocity * self.dt
    return feedbackForce - extraDampingForce
