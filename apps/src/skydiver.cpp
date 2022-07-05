// From http://cpp.api.tudat.space/tutorials/basics/externalLibraries/utilityExamples.html

#include <functional>
#include <Eigen/Core>
#include "tudat/math/integrators/rungeKutta4Integrator.h"

class Skydiver
{
public:

    //! Constructor of the Skydiver class.
    Skydiver( ){ }

    //! Compute the state derivative for given time and state.
    Eigen::Vector2d computeStateDerivative( const double time, const Eigen::Vector2d& state );

    //! Compute the final state for given initial conditions, final time and time step-size.
    Eigen::Vector2d computeFinalState( const double initialTime, const Eigen::Vector2d& initialState,
                                       const double endTime, const double timeStep );

protected:

private:

};

Eigen::Vector2d Skydiver::computeStateDerivative( const double time, const Eigen::Vector2d& state )
{
    // Note that the time is not used in this function, but it is required as input for the integrator.
    Eigen::Vector2d stateDerivative = Eigen::Vector2d::Zero( ); // Declare and initialize to zero.
    stateDerivative( 0 ) = state( 1 );  // Velocity
    stateDerivative( 1 ) = -9.81;       // Acceleration

    return stateDerivative;
}

Eigen::Vector2d Skydiver::computeFinalState( const double initialTime, const Eigen::Vector2d& initialState,
                                             const double endTime, const double timeStep )
{
    tudat::numerical_integrators::RungeKutta4IntegratorXd integrator(
            std::bind( &Skydiver::computeStateDerivative, this,
                       std::placeholders::_1, std::placeholders::_2 ),
            initialTime, initialState );

    return integrator.integrateTo( endTime, timeStep );
}

int main()
{
    double initialTime = 0;
    double timeStep = 0.01;
    double endTime = 40;
    Eigen::Vector2d initialState(25e3, 0);

    Skydiver testDiver;
    const Eigen::Vector2d endState = testDiver.computeFinalState( initialTime, initialState, endTime, timeStep );

    std::cout << endState << "\n";

    return 0;
}
