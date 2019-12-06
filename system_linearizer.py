import numpy

PARAMETERS_NUMBER = 3
INPUTS_NUMBER = 3
SAMPLES_NUMBER = 200

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
    self.inputsList[ sampleIndex ][ 2 ] = inputForce + outputForce
    self.samplesCount += 1
  
  def IdentifySystem( self, defaultImpedance ):
    inputImpedance = defaultImpedance
    outputImpedance = defaultImpedance
    plantImpedance = defaultImpedance
    if self.samplesCount >= SAMPLES_NUMBER:
      parameters, residuals, rank, s = numpy.linalg.lstsq( self.statesList, self.inputsList, rcond=None )
      inputImpedance = numpy.maximum( ( parameters[ 0 ][ 0 ], parameters[ 1 ][ 0 ], parameters[ 2 ][ 0 ] ), ( 0.0, 0.0, 0.0 ) )
      outputImpedance = numpy.maximum( ( parameters[ 0 ][ 1 ], parameters[ 1 ][ 1 ], parameters[ 2 ][ 1 ] ), ( 0.0, 0.0, 0.0 ) )
      plantImpedance = numpy.maximum( ( parameters[ 0 ][ 2 ], parameters[ 1 ][ 2 ], parameters[ 2 ][ 2 ] ), ( 0.0, 0.0, 0.0 ) )
    
    return ( inputImpedance, outputImpedance, plantImpedance )
