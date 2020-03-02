import math
from matplotlib import pyplot

def plotData( timeSteps, referencePositions, masterPositions, slavePositions, masterVelocities, slaveVelocities, masterDelayedPositions, slaveDelayedPositions, masterSetpointPositions, slaveSetpointPositions, masterInputs, slaveInputs, masterFeedbackInputs, slaveFeedbackInputs, masterInputInertias, slaveOutputInertias, masterInputDampings, slaveOutputDampings, masterInputStiffnesses, slaveOutputStiffnesses, masterInputEnergy, slaveFeedbackEnergy, masterNetEnergy, masterDissipatedEnergy, referenceEnergy ):
  
  # perfomance calculation
  positionErrorRMS = 0.0
  velocityErrorRMS = 0.0
  forceErrorRMS = 0.0
  inertiaErrorRMS = 0.0
  dampingErrorRMS = 0.0
  stiffnessErrorRMS = 0.0
  timeStepsNumber = len( timeSteps )
  for stepIndex in range( timeStepsNumber ):
    positionErrorRMS += ( masterPositions[ stepIndex ] - slavePositions[ stepIndex ] )**2 / timeStepsNumber
    velocityErrorRMS += ( masterVelocities[ stepIndex ] - slaveVelocities[ stepIndex ] )**2 / timeStepsNumber
    forceErrorRMS += ( masterInputs[ stepIndex ] - masterFeedbackInputs[ stepIndex ] )**2 / timeStepsNumber
    inertiaErrorRMS += ( masterInputInertias[ stepIndex ] - slaveOutputInertias[ stepIndex ] )**2 / timeStepsNumber
    dampingErrorRMS += ( masterInputDampings[ stepIndex ] - slaveOutputDampings[ stepIndex ] )**2 / timeStepsNumber
    stiffnessErrorRMS += ( masterInputStiffnesses[ stepIndex ] - slaveOutputStiffnesses[ stepIndex ] )**2 / timeStepsNumber
  
  positionErrorRMS = math.sqrt( positionErrorRMS )
  velocityErrorRMS = math.sqrt( velocityErrorRMS )
  forceErrorRMS = math.sqrt( forceErrorRMS )
  inertiaErrorRMS = math.sqrt( inertiaErrorRMS )
  dampingErrorRMS = math.sqrt( dampingErrorRMS )
  stiffnessErrorRMS = math.sqrt( stiffnessErrorRMS )
  
  print( "{:.3f} {:.3f} {:.3f} {:.3f} {:.3f} {:.3f}".format( positionErrorRMS, inertiaErrorRMS, dampingErrorRMS, stiffnessErrorRMS, masterInputEnergy[ -1 ] - masterDissipatedEnergy[ -1 ] - masterNetEnergy[ -1 ], velocityErrorRMS + forceErrorRMS ) )
  
  pyplot.subplot( 411, xlim=[ 0.0, timeSteps[ -1 ] ], ylim=[ -0.8, 0.8 ] )
  #pyplot.title( 'Teleoperation w/ Interactive Environment (delay={}±{}[s])\n{} Controller ( position RMS error={:.3f}, impedance RMS error=({:.3f},{:.3f},{:.3f}) )\n'.format( 
  #              NET_DELAY_AVG, NET_DELAY_VAR, CONTROLLER_NAMES[ controllerType ], positionErrorRMS, inertiaErrorRMS, dampingErrorRMS, stiffnessErrorRMS ), fontsize=15 )
  pyplot.ylabel( 'Position [m]', fontsize=15 )
  pyplot.tick_params( axis='x', labelsize=0 )
  pyplot.plot( timeSteps, referencePositions, 'k:' )
  #pyplot.plot( timeSteps, masterDelayedPositions, 'b:', timeSteps, slaveDelayedPositions, 'r:' )
  #pyplot.plot( timeSteps, masterSetpointPositions, 'b--', timeSteps, slaveSetpointPositions, 'r--' )
  pyplot.plot( timeSteps, masterPositions, 'b-', timeSteps, slavePositions, 'r-' )
  pyplot.legend( [ 'reference', 'master', 'slave' ] )
  #pyplot.legend( [ 'reference', 'master-delayed', 'slave-delayed', 'master-setpoint', 'slave-setpoint', 'master', 'slave' ] )
  #pyplot.legend( [ 'master-delayed', 'slave-delayed', 'master-setpoint', 'slave-setpoint', 'master', 'slave' ] )
  
  pyplot.subplot( 412, xlim=[ 0.0, timeSteps[ -1 ] ], ylim=[ -10, 10 ] )
  pyplot.ylabel( 'Force [N]', fontsize=15 )
  #pyplot.tick_params( axis='x', labelsize=0 )
  pyplot.plot( timeSteps, masterInputs, 'g-', timeSteps, slaveInputs, 'm-' )
  pyplot.plot( timeSteps, masterFeedbackInputs, 'b-', timeSteps, slaveFeedbackInputs, 'r-' )
  pyplot.legend( [ 'master-input', 'slave-input', 'master-feedback', 'slave-feedback' ] )
  pyplot.subplot( 413, xlim=[ 0.0, timeSteps[ -1 ] ], ylim=[ -2.0, 8.0 ] )
  pyplot.ylabel( 'Energy [J]', fontsize=15 )
  pyplot.tick_params( axis='x', labelsize=0 )
  pyplot.plot( timeSteps, masterInputEnergy, 'b-', timeSteps, slaveFeedbackEnergy, 'r-', timeSteps, masterDissipatedEnergy, 'm-', timeSteps, masterNetEnergy, 'g-' )
  #pyplot.plot( timeSteps, referenceEnergy, 'k:' )
  pyplot.legend( [ 'master-input', 'slave-feeback', 'master-dissipated', 'master-resulting' ] )
  
  pyplot.subplot( 414, xlim=[ 0.0, timeSteps[ -1 ] ], ylim=[ -0.5, 7.5 ] )
  pyplot.ylabel( 'Impedance', fontsize=15 )
  pyplot.xlabel( 'Time [s]', fontsize=15 )
  pyplot.plot( timeSteps, masterInputInertias, 'b--', timeSteps, slaveOutputInertias, 'r--' )
  pyplot.plot( timeSteps, masterInputDampings, 'b-.', timeSteps, slaveOutputDampings, 'r-.' )
  pyplot.plot( timeSteps, masterInputStiffnesses, 'b:', timeSteps, slaveOutputStiffnesses, 'r:' )
  pyplot.legend( [ 'master-inertia', 'slave-inertia', 'master-damping', 'slave-damping', 'master-stiffness', 'slave-stiffness' ] )
  pyplot.show()
