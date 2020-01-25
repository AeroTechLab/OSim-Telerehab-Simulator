from wave_controller import WaveController
from kalman_filter import KalmanFilter

class WavePredTeleoperator:
  
  remotePosition = 0.0
  outputWaveIntegral = 0.0
  
  def __init__( self, impedance, timeStep ):
    self.waveController = WaveController( impedance[ 0 ] + impedance[ 1 ] + impedance[ 2 ], timeStep )
    self.dt = timeStep
    self.waveObserver = KalmanFilter( 2, 2 )
    self.waveObserver.SetMeasurement( 0, 0, 4.0 )
    self.waveObserver.SetMeasurement( 1, 1, 4.0 )
    self.waveObserver.SetStatePredictionFactor( 0, 1, self.dt )
  
  def SetSystem( self, impedance ):
    waveImpedance = ( max( impedance[ 0 ], 0.0 ), max( impedance[ 1 ], 1.0 ), max( impedance[ 2 ], 0.0 ) )
    self.waveController.SetImpedance( waveImpedance[ 0 ] + waveImpedance[ 1 ] + waveImpedance[ 2 ] )
  
  def Process( self, localState, localForce, remotePacket, timeDelay ):
    
    localPosition, localVelocity, localAcceleration = localState
    inputWave, inputWaveIntegral, inputEnergy, remoteImpedance = remotePacket
    
    remoteState, predictedWave = self.waveObserver.Process( [ inputWaveIntegral + inputWave * timeDelay, inputWave ] )
    
    remoteVelocity = self.waveController.PreProcess( predictedWave[ 1 ], inputEnergy, remoteImpedance )
    feedbackForce, outputWave, outputEnergy, localImpedance = self.waveController.PostProcess( localVelocity, localForce )
    
    self.remotePosition += remoteVelocity * self.dt
    self.outputWaveIntegral += outputWave * self.dt
    
    return ( feedbackForce, ( self.remotePosition, remoteVelocity, 0.0 ), ( outputWave, self.outputWaveIntegral, outputEnergy, localImpedance ) )
