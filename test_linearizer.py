import numpy

from system_linearizer import SystemLinearizer

linearizer = SystemLinearizer()

data = numpy.loadtxt( 'samples-3.txt' )
samplesNumber = data.shape[ 0 ]

inputImpedance = []
outputImpedance = []
plantImpedance = []
sampleImpedance = []

for index in range( samplesNumber - 1 ):
  force = data[ index, 1 ] + data[ index, 5 ]
  position = data[ index, 2 ]
  velocity = data[ index, 3 ]
  acceleration = data[ index, 4 ]
  sampleImpedance.append( ( data[ index, 6 ], data[ index, 7 ], data[ index, 8 ] ) )
  linearizer.AddSample( position, velocity, acceleration, force, 0.0 )
  impedance = linearizer.IdentifySystem( ( 0.0, 0.0, 0.0 ) )
  inputImpedance.append( impedance[ 0 ] )
  outputImpedance.append( impedance[ 1 ] )
  plantImpedance.append( impedance[ 2 ] )
  print( impedance[ 0 ], sampleImpedance[ -1 ] )
  
print()
print( numpy.average( inputImpedance, axis=0 ) )
print( numpy.average( sampleImpedance, axis=0 ) )
