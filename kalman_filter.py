import numpy

class KalmanFilter:
  def __init__( self, statesNumber, measuresNumber, inputsNumber=1 ):
    self.state = numpy.array( [ 0.0 for value in range( statesNumber ) ] )
    self.observer = numpy.zeros( ( measuresNumber, statesNumber ) )
    self.statePredictor = numpy.eye( statesNumber )
    self.inputPredictor = numpy.zeros( ( statesNumber, inputsNumber ) )
    self.predictionCovariance = numpy.eye( statesNumber )
    self.predictionCovarianceNoise = numpy.eye( statesNumber )
    self.errorCovarianceNoise = numpy.eye( measuresNumber )

  def SetMeasurement( self, measureIndex, stateIndex, deviation ):
    self.observer[ measureIndex ][ stateIndex ] = 1.0
    self.errorCovarianceNoise[ measureIndex ][ measureIndex ] = deviation ** 2    

  def SetStatePredictionFactor( self, newStateIndex, oldStateIndex, ratio ):
    self.statePredictor[ newStateIndex, oldStateIndex ] = ratio
    
  def SetInputPredictionFactor( self, newStateIndex, inputIndex, ratio ):
    self.inputPredictor[ newStateIndex, inputIndex ] = ratio
    
  def SetObservationFactor( self, measureIndex, stateIndex, ratio ):
    self.observer[ measureIndex, stateIndex ] = ratio
  
  def Update( self, measures, inputs=[ 0.0 ] ):
    self.state = self.statePredictor.dot( self.state ) + self.inputPredictor.dot( inputs )
    
    self.predictionCovariance = self.statePredictor.dot( self.predictionCovariance ).dot( self.statePredictor.T )
    self.predictionCovariance = self.predictionCovariance + self.predictionCovarianceNoise
    
    estimatedMeasures = self.observer.dot( self.state )
    error = numpy.array( measures ) - estimatedMeasures
    
    errorCovariance = self.observer.dot( self.predictionCovariance ).dot( self.observer.T )
    errorCovariance = errorCovariance + self.errorCovarianceNoise
    
    gain = self.predictionCovariance.dot( self.observer.T ).dot( numpy.linalg.inv( errorCovariance ) )
    
    self.state = self.state + gain.dot( error )
    self.predictionCovariance = self.predictionCovariance - gain.dot( self.observer ).dot( self.predictionCovariance )
    
    return numpy.ravel( self.state ).tolist(), numpy.ravel( estimatedMeasures ).tolist()
