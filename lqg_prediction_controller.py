from lqg_controller import LQGController 
from kalman_filter import KalmanFilter 

class LQGPredController:
  measurement = [ 0.0, 0.0, 0.0 ]
  controlForce = 0.0
  
  def __init__( self, inertia, damping, stiffness, timeStep ):
    self.dt = timeStep
    self.localController = LQGController( inertia, damping, stiffness, timeStep )
    self.remoteStateObserver = KalmanFilter( 3, 3 )
    self.remoteStateObserver.SetMeasurement( 0, 0, 4.0 )
    self.remoteStateObserver.SetMeasurement( 1, 1, 4.0 )
    self.remoteStateObserver.SetMeasurement( 2, 2, 4.0 )
    self.remoteInputObserver = KalmanFilter( 3, 1 )
    self.remoteInputObserver.SetMeasurement( 0, 0, 1.0 )
  
  def SetSystem( self, inertia, damping, stiffness ):
    self.localController.SetSystem( inertia, damping, stiffness )
  
  def Predict( self, remoteMeasurement, remoteForce, timeDelay ):
    timeDelay = int( timeDelay / self.dt ) * self.dt
    predictedState = list( remoteMeasurement )
    predictedState[ 0 ] += predictedState[ 1 ] * timeDelay + predictedState[ 2 ] * 0.5 * timeDelay**2
    predictedState[ 1 ] += predictedState[ 2 ] * timeDelay
    remoteState, predictedState = self.remoteStateObserver.Process( predictedState )
    remoteState, predictedForce = self.remoteInputObserver.Predict()
    remoteState, predictedForce = self.remoteInputObserver.Update( [ remoteForce ], [ remoteState[ 0 ] ] )
    predictedForce[ 0 ] += remoteState[ 1 ] * timeDelay + remoteState[ 2 ] * 0.5 * timeDelay**2
    return ( tuple( predictedState ), predictedForce[ 0 ] )

  def Process( self, setpoint, measurement, externalForce ):
    return self.localController.Process( setpoint, measurement, externalForce )
