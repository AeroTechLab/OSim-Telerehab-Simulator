import numpy

class SimplePlant:
  def __init__( self, inertia, damping, stiffness, timeStep ):
    self.state = numpy.array( [ 0.0, 0.0, 0.0 ] )
    self.statePredictor = numpy.eye( 3 )
    self.statePredictor[ 0 ][ 1 ] = timeStep
    self.statePredictor[ 0 ][ 2 ] =  0.5 * timeStep**2
    self.statePredictor[ 1 ][ 2 ] = timeStep
    self.statePredictor[ 2 ][ 0 ] = -stiffness / inertia
    self.statePredictor[ 2 ][ 1 ] = -damping / inertia
    self.statePredictor[ 2 ][ 2 ] = 0.0
    self.inputPredictor = numpy.zeros( ( 3, 1 ) )
    self.inputPredictor[ 2 ][ 0 ] = 1 / inertia
  
  def Process( self, input ):
    self.state = self.statePredictor.dot( self.state ) + self.inputPredictor.dot( [ input ] )
    return ( self.state[ 0 ], self.state[ 1 ], self.state[ 2 ] )
