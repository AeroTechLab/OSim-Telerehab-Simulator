from random import randrange as rand
from math import exp

import numpy

class MLPerceptron:
  
  def __init__( self, inputsNumber, outputsNumber, hiddenNumber, precision=10e-6, learningRate=0.1, maxMomentum=0.5 ):
    self.inputWeightsTable = [ [ rand( 1001 ) / 1000 for inputIndex in range( inputsNumber + 1 ) ] for neuronIndex in range( hiddenNumber ) ]
    self.outputWeightsTable = [ [ rand( 1001 ) / 1000 for neuronIndex in range( hiddenNumber + 1 ) ] for outputIndex in range( outputsNumber ) ]
    self.inputWeightDeltasTable = [ [ 0.0 for inputIndex in range( inputsNumber + 1 ) ] for neuronIndex in range( hiddenNumber ) ]
    self.outputWeightDeltasTable = [ [ 0.0 for neuronIndex in range( hiddenNumber + 1 ) ] for outputIndex in range( outputsNumber ) ]
    self.inputLimitsTable = [ [ limit for i in range( inputsNumber ) ] for limit in [ -1.0, +1.0 ] ]
    self.outputLimitsTable = [ [ limit for i in range( outputsNumber ) ] for limit in [ -1.0, +1.0 ] ]
    self.precision = precision
    self.learningRate = learningRate if ( learningRate > 0.0 and learningRate <= 0.2 ) else 0.1
    self.maxMomentum = maxMomentum if ( maxMomentum >= 0.0 and maxMomentum <= 1.0 ) else 0.5
    
  def GetRanges( self, inputsTable, outputsTable ):
    self.inputLimitsTable[ 0 ] = numpy.amin( inputsTable, axis=0 ).tolist()
    self.inputLimitsTable[ 1 ] = numpy.amax( inputsTable, axis=0 ).tolist()
    self.outputLimitsTable[ 0 ] = numpy.amin( outputsTable, axis=0 ).tolist()
    self.outputLimitsTable[ 1 ] = numpy.amax( outputsTable, axis=0 ).tolist()

  def Normalize( self, valuesList, valueLimitsTable ):
    return list( map( lambda value,min,max: ( value - min ) / ( max - min ), valuesList, valueLimitsTable[ 0 ], valueLimitsTable[ 1 ] ) )
      
  def Denormalize( self, valuesList, valueLimitsTable ):
    return list( map( lambda value,min,max: value * ( max - min ) + min, valuesList, valueLimitsTable[ 0 ], valueLimitsTable[ 1 ] ) )

  def GetLayerActivations( self, inputsList, weightsTable ):
    neuronActivations = numpy.dot( weightsTable, inputsList )
    return neuronActivations.tolist()

  def GetLayerOutputs( self, inputsList, weightsTable ):
    layerActivations = self.GetLayerActivations( inputsList, weightsTable )
    return list( map( lambda activation: 1 / ( 1 + exp( -activation ) ), layerActivations ) )

  def GetLayerOutputDerivatives( self, inputsList, weightsTable ):
    layerActivations = self.GetLayerActivations( inputsList, weightsTable )
    return list( map( lambda activation: exp( activation ) / ( ( 1 + exp( activation ) ) ** 2 ), layerActivations ) )

  def Process( self, inputsList ):
    #print( inputsList )
    internalInputsList = self.Normalize( inputsList, self.inputLimitsTable )
    internalInputsList.append( -1 )
    hiddenNeuronOutputs = self.GetLayerOutputs( internalInputsList, self.inputWeightsTable )
    hiddenNeuronOutputs.append( -1 )
    internalOutputsList = self.GetLayerOutputs( hiddenNeuronOutputs, self.outputWeightsTable )
    return self.Denormalize( internalOutputsList, self.outputLimitsTable )

  def PostProcess( self, outputsList ):
    return list( map( lambda x: 0 if x < 0.5 else 1 , outputsList ) )

  def Validate( self, inputSamplesTable, outputSamplesTable ):
    averageSquareError = 0
    for ( inputSamplesList, outputSamplesList ) in zip( inputSamplesTable, outputSamplesTable ):
      outputsList = self.Normalize( self.Process( inputSamplesList ), self.outputLimitsTable )
      outputSamplesList = self.Normalize( outputSamplesList, self.outputLimitsTable )
      for ( output, outputSample ) in zip( outputsList, outputSamplesList ):
        outputError = outputSample - output
        averageSquareError += ( outputError * outputError ) / len( outputSamplesTable )
    return averageSquareError

  def Train( self, inputSamplesTable, outputSamplesTable, maxEpochsNumber ):
    epochsNumber = 0
    lastAverageSquareError = 0.0
    nextMomentum = 0.0
    averageSquareErrors = []
    self.GetRanges( inputSamplesTable, outputSamplesTable )
    while True:
      for ( inputSamplesList, outputSamplesList ) in zip( inputSamplesTable, outputSamplesTable ):
        for ( inputWeightsList, inputWeightDeltasList ) in zip( self.inputWeightsTable, self.inputWeightDeltasTable ):
          for inputIndex in range( len( inputWeightsList ) - 1 ):
            inputWeightDeltasList[ inputIndex ] = nextMomentum * inputWeightDeltasList[ inputIndex ]
            inputWeightsList[ inputIndex ] += inputWeightDeltasList[ inputIndex ]
        for ( outputWeightsList, outputWeightDeltasList ) in zip( self.outputWeightsTable, self.outputWeightDeltasTable ):
          for neuronIndex in range( len( outputWeightsList ) - 1 ):
            outputWeightDeltasList[ neuronIndex ] = nextMomentum * outputWeightDeltasList[ neuronIndex ]
            outputWeightsList[ neuronIndex ] += outputWeightDeltasList[ neuronIndex ]
        inputsList = self.Normalize( inputSamplesList, self.inputLimitsTable )
        inputsList.append( -1 )
        hiddenNeuronOutputs = self.GetLayerOutputs( inputsList, self.inputWeightsTable )
        hiddenNeuronOutputs.append( -1 )
        outputsList = self.GetLayerOutputs( hiddenNeuronOutputs, self.outputWeightsTable )
        outputDerivatives = self.GetLayerOutputDerivatives( hiddenNeuronOutputs, self.outputWeightsTable )
        outputDeltas = []
        referenceOutputsList = self.Normalize( outputSamplesList, self.outputLimitsTable )
        for outputIndex in range( len( self.outputWeightsTable ) ):
          outputError = referenceOutputsList[ outputIndex ] - outputsList[ outputIndex ]
          outputDeltas.append( outputError * outputDerivatives[ outputIndex ] )
          for neuronIndex in range( len( hiddenNeuronOutputs ) - 1 ):
            self.outputWeightDeltasTable[ outputIndex ][ neuronIndex ] += self.learningRate * outputDeltas[ outputIndex ] * hiddenNeuronOutputs[ neuronIndex ]
            self.outputWeightsTable[ outputIndex ][ neuronIndex ] += self.outputWeightDeltasTable[ outputIndex ][ neuronIndex ]
          self.outputWeightsTable[ outputIndex ][ -1 ] += self.learningRate * outputDeltas[ outputIndex ] * hiddenNeuronOutputs[ -1 ]
        hiddenNeuronDerivatives = self.GetLayerOutputDerivatives( inputsList, self.inputWeightsTable )
        for neuronIndex in range( len( hiddenNeuronDerivatives ) ):
          inputDelta = 0
          for outputIndex in range( len( self.outputWeightsTable ) ):
            inputDelta += outputDeltas[ outputIndex ] * hiddenNeuronDerivatives[ neuronIndex ] * self.outputWeightsTable[ outputIndex ][ neuronIndex ]
          for inputIndex in range( len( inputsList ) - 1 ):
            self.inputWeightDeltasTable[ neuronIndex ][ inputIndex ] += self.learningRate * inputDelta * inputsList[ inputIndex ]
            self.inputWeightsTable[ neuronIndex ][ inputIndex ] += self.inputWeightDeltasTable[ neuronIndex ][ inputIndex ]  
          self.inputWeightsTable[ neuronIndex ][ -1 ] += self.learningRate * inputDelta * inputsList[ -1 ]
      epochsNumber += 1
      nextMomentum = ( nextMomentum + self.maxMomentum ) / 2
      averageSquareError = self.Validate( inputSamplesTable, outputSamplesTable )
      averageSquareErrors.append( averageSquareError )
      if abs( lastAverageSquareError - averageSquareError ) < self.precision: break
      if epochsNumber > maxEpochsNumber: break
      lastAverageSquareError = averageSquareError
    return ( epochsNumber, averageSquareErrors )

  def GetWeights( self ):
    return ( self.inputWeightsTable, self.outputWeightsTable, self.inputLimitsTable, self.outputLimitsTable )

  def SetWeights( self, inputWeightsTable, outputWeightsTable, inputLimitsTable, outputLimitsTable ):
    self.inputWeightsTable = [ line[ : ] for line in inputWeightsTable ]
    self.outputWeightsTable = [ line[ : ] for line in outputWeightsTable ]
    self.inputWeightDeltasTable = [ [ 0.0 for column in line ] for line in inputWeightsTable ]
    self.outputWeightDeltasTable = [ [ 0.0 for column in line ] for line in outputWeightsTable ]
    self.inputLimitsTable = [ inputLimitsTable[ limit ][ : ] for limit in range( 2 ) ]
    self.outputLimitsTable = [ outputLimitsTable[ limit ][ : ] for limit in range( 2 ) ]
