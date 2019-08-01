import numpy

class KalmanFilter:
  def __init__( self, statesNumber, measuresNumber, inputsNumber=1 ):
    self.state = numpy.array( [ 0.0 for value in range( statesNumber ) ] )
    self.error = numpy.array( [ 0.0 for value in range( measuresNumber ) ] )
    self.observer = numpy.zeros( ( measuresNumber, statesNumber ) )
    self.statePredictor = numpy.eye( statesNumber )
    self.inputPredictor = numpy.zeros( ( statesNumber, inputsNumber ) )
    self.predictionCovariance = numpy.eye( statesNumber )
    self.predictionCovarianceNoise = numpy.eye( statesNumber )
    self.errorCovarianceNoise = numpy.eye( measuresNumber )

  def SetMeasurement( self, measureIndex, stateIndex, deviation ):
    self.observer[ measureIndex ][ stateIndex ] = 1.0
    self.errorCovarianceNoise[ measureIndex ][ measureIndex ] = deviation**2    

  def SetStatePredictionFactor( self, newStateIndex, preStateIndex, ratio ):
    self.statePredictor[ newStateIndex, preStateIndex ] = ratio
    
  def SetInputPredictionFactor( self, newStateIndex, inputIndex, ratio ):
    self.inputPredictor[ newStateIndex, inputIndex ] = ratio
    
  def SetObservationFactor( self, measureIndex, stateIndex, ratio ):
    self.observer[ measureIndex, stateIndex ] = ratio
    
  def SetPredictionNoise( self, stateIndex, deviation ):
    self.predictionCovarianceNoise[ stateIndex ][ stateIndex ] = deviation**2 
  
  def Predict( self, inputs=[ 0.0 ] ):
    self.state = self.statePredictor.dot( self.state ) + self.inputPredictor.dot( inputs )
    
    self.predictionCovariance = self.statePredictor.dot( self.predictionCovariance ).dot( self.statePredictor.T )
    self.predictionCovariance = self.predictionCovariance + self.predictionCovarianceNoise
    
    estimatedMeasures = self.observer.dot( self.state )
    
    return numpy.ravel( self.state ).tolist(), numpy.ravel( estimatedMeasures ).tolist()
  
  def Update( self, measures, estimatedMeasures ):
    self.error = numpy.array( measures ) - estimatedMeasures
    
    errorCovariance = self.observer.dot( self.predictionCovariance ).dot( self.observer.T )
    errorCovariance = errorCovariance + self.errorCovarianceNoise
    
    self.gain = self.predictionCovariance.dot( self.observer.T ).dot( numpy.linalg.inv( errorCovariance ) )
    
    self.state = self.state + self.gain.dot( self.error )
    self.predictionCovariance = self.predictionCovariance - self.gain.dot( self.observer ).dot( self.predictionCovariance )
    
    estimatedMeasures = self.observer.dot( self.state )
    
    return numpy.ravel( self.state ).tolist(), numpy.ravel( estimatedMeasures ).tolist()
  
  def Process( self, measures, inputs=[ 0.0 ] ):
    estimatedState, estimatedMeasures = self.Predict( inputs )
    return self.Update( measures, estimatedMeasures )
