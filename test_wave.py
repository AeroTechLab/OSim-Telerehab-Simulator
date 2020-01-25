import math

waveImpedance = 1.0
feedbackForce = 0.0
bandwidth = 0.5
filteredWave = 0.0
lastInputWave = 0.0

setpoints = [ 0, 1, 2, 3, 4, 5, 4, 3, 2, 1, 0 ]
setpointDerivatives = [ 1, 1, 1, 1, 1, 0, -1, -1, -1, -1, -1 ]
for (setpoint,derivative) in zip(setpoints,setpointDerivatives):
  inputWave = ( waveImpedance * derivative - feedbackForce ) / math.sqrt( 2.0 * waveImpedance )
  print( inputWave )
  remotePosition = setpoint
  positionError = 0.0 - remotePosition
  waveCorrection = - math.sqrt( 2.0 * waveImpedance ) * bandwidth * positionError
  if positionError * inputWave < 0: waveCorrection = 0
  elif abs( waveCorrection ) > abs( inputWave ): waveCorrection = - inputWave
  inputWave += waveCorrection
  print( inputWave )
  filteredWave = ( ( 2 - bandwidth ) * filteredWave + bandwidth * ( inputWave + lastInputWave ) ) / ( 2 + bandwidth )
  lastInputWave = inputWave 
  print( filteredWave )
  feedbackForce = - ( waveImpedance * 0.0 - math.sqrt( 2.0 * waveImpedance ) * filteredWave )
  print( feedbackForce )
  outputWave = ( waveImpedance * 0.0 - feedbackForce ) / math.sqrt( 2.0 * waveImpedance )  
  print( outputWave )
  print()
