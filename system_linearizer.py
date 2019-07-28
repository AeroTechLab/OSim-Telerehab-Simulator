import numpy
#import scipy.optimize as optimization

PARAMETERS_NUMBER = 3
INPUTS_NUMBER = 2
SAMPLES_NUMBER = 100

class SystemLinearizer:
  samplesCount = 0

  def __init__( self ):
    self.statesList = numpy.zeros( ( SAMPLES_NUMBER, PARAMETERS_NUMBER ) )
    self.inputsList = numpy.zeros( ( SAMPLES_NUMBER, INPUTS_NUMBER ) )
    
  def AddSample( self, position, velocity, acceleration, inputForce, outputForce ):
    sampleIndex = self.samplesCount % SAMPLES_NUMBER
    self.statesList[ sampleIndex ][ 0 ] = acceleration
    self.statesList[ sampleIndex ][ 1 ] = velocity
    self.statesList[ sampleIndex ][ 2 ] = position
    self.inputsList[ sampleIndex ][ 0 ] = inputForce
    self.inputsList[ sampleIndex ][ 1 ] = outputForce
    self.samplesCount += 1
  
  def IdentifySystem( self, defaultImpedance ):
    inputImpedance = defaultImpedance
    outputImpedance = defaultImpedance
    if self.samplesCount >= SAMPLES_NUMBER:
      estimatedParameters, residuals, rank, s = numpy.linalg.lstsq( self.statesList, self.inputsList, rcond=None )
      #estimatedParameters = optimization.nnls( inputSamplesTable, outputSamplesList.ravel() )[ 0 ]
      #estimatedParameters = optimization.lsq_linear( inputSamplesTable, outputSamplesList.ravel(), bounds=( 0, numpy.inf ) ).x
      inputImpedance = ( estimatedParameters[ 0 ][ 0 ],  estimatedParameters[ 1 ][ 0 ], estimatedParameters[ 2 ][ 0 ] )
      outputImpedance = ( estimatedParameters[ 0 ][ 1 ],  estimatedParameters[ 1 ][ 1 ], estimatedParameters[ 2 ][ 1 ] )
    
    return ( inputImpedance, outputImpedance )
