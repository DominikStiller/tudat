/*    Copyright (c) 2010-2017, Delft University of Technology
 *    All rigths reserved
 *
 *    This file is part of the Tudat. Redistribution and use in source and
 *    binary forms, with or without modification, are permitted exclusively
 *    under the terms of the Modified BSD license. You should have received
 *    a copy of the license with this file. If not, please or visit:
 *    http://tudat.tudelft.nl/LICENSE.
 */

#define BOOST_TEST_MAIN

#include <Tudat/SimulationSetup/tudatEstimationHeader.h>
#include "Tudat/SimulationSetup/PropagationSetup/fullPropagationLambertTargeter.h"
#include "Tudat/SimulationSetup/PropagationSetup/fullPropagationMGAtrajectory.h"
#include <boost/test/floating_point_comparison.hpp>
#include <boost/test/unit_test.hpp>
#include <Eigen/Core>
#include "Tudat/Basics/testMacros.h"


// required for the MGA
#include "Tudat/Astrodynamics/Ephemerides/approximatePlanetPositions.h"
#include "Tudat/Astrodynamics/TrajectoryDesign/trajectory.h"
#include "Tudat/Astrodynamics/TrajectoryDesign/exportTrajectory.h"
#include "Tudat/Astrodynamics/TrajectoryDesign/planetTrajectory.h"


namespace tudat
{

namespace unit_tests
{

using namespace tudat;

//! Test of the full propagation of a trajectory
BOOST_AUTO_TEST_SUITE( testFullPropagationTrajectory )

//! Test of the full propagation of a MGA trajectory
BOOST_AUTO_TEST_CASE( testFullPropagationMGA )
{

    std::cout << "Cassini trajectory: " << "\n\n";

    std::cout.precision(20);

    double initialTime = 0.0;
    double fixedStepSize = 1000.0;

    // Define integrator settings.
    std::shared_ptr< numerical_integrators::IntegratorSettings< double > > integratorSettings =
            std::make_shared < numerical_integrators::IntegratorSettings < > >
                ( numerical_integrators::rungeKutta4, initialTime, fixedStepSize);

    // Specify required parameters
    // Specify the number of legs and type of legs.
    int numberOfLegs = 6;
    std::vector< transfer_trajectories::TransferLegType > legTypeVector;
    legTypeVector.resize( numberOfLegs );
    legTypeVector[ 0 ] = transfer_trajectories::mga_Departure;
    legTypeVector[ 1 ] = transfer_trajectories::mga_Swingby;
    legTypeVector[ 2 ] = transfer_trajectories::mga_Swingby;
    legTypeVector[ 3 ] = transfer_trajectories::mga_Swingby;
    legTypeVector[ 4 ] = transfer_trajectories::mga_Swingby;
    legTypeVector[ 5 ] = transfer_trajectories::capture;

    // Name of the bodies involved in the trajectory
    std::vector< std::string > nameBodiesTrajectory;
    nameBodiesTrajectory.push_back("Earth");
    nameBodiesTrajectory.push_back("Venus");
    nameBodiesTrajectory.push_back("Venus");
    nameBodiesTrajectory.push_back("Earth");
    nameBodiesTrajectory.push_back("Jupiter");
    nameBodiesTrajectory.push_back("Saturn");



    std::vector< std::string > centralBody;
    centralBody.push_back( "Sun" );
    std::vector< std::string > bodyToPropagate;
    bodyToPropagate.push_back( "spacecraft" );


    spice_interface::loadStandardSpiceKernels( );



    std::map< std::string, std::shared_ptr< simulation_setup::BodySettings > > bodySettings =
                    simulation_setup::getDefaultBodySettings( centralBody );

    // Define central body ephemeris settings.
    std::string frameOrigin = "SSB";
    std::string frameOrientation = "J2000";
    bodySettings[ centralBody[0] ]->ephemerisSettings = std::make_shared< simulation_setup::ConstantEphemerisSettings >(
            ( Eigen::Vector6d( ) << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ).finished( ), frameOrigin, frameOrientation );

    bodySettings[ centralBody[0] ]->ephemerisSettings->resetFrameOrientation( frameOrientation );
    bodySettings[ centralBody[0] ]->rotationModelSettings->resetOriginalFrame( frameOrientation );



    // Create body map.
    simulation_setup::NamedBodyMap bodyMap = createBodies( bodySettings );

    // ephemeris
    bodyMap["Earth"] = std::make_shared< simulation_setup::Body >( );
    bodyMap["Earth"]->setEphemeris( std::make_shared< ephemerides::ApproximatePlanetPositions >(
                                        ephemerides::ApproximatePlanetPositionsBase::BodiesWithEphemerisData::earthMoonBarycenter ));
    bodyMap["Venus"] = std::make_shared< simulation_setup::Body >( );
    bodyMap["Venus"]->setEphemeris( std::make_shared< ephemerides::ApproximatePlanetPositions >(
                                        ephemerides::ApproximatePlanetPositionsBase::BodiesWithEphemerisData::venus ));
    bodyMap["Jupiter"] = std::make_shared< simulation_setup::Body >( );
    bodyMap["Jupiter"]->setEphemeris( std::make_shared< ephemerides::ApproximatePlanetPositions >(
                                        ephemerides::ApproximatePlanetPositionsBase::BodiesWithEphemerisData::jupiter ));
    bodyMap["Saturn"] = std::make_shared< simulation_setup::Body >( );
    bodyMap["Saturn"]->setEphemeris( std::make_shared< ephemerides::ApproximatePlanetPositions >(
                                        ephemerides::ApproximatePlanetPositionsBase::BodiesWithEphemerisData::saturn ));


    // gravity field
    bodyMap["Earth"]->setGravityFieldModel( simulation_setup::createGravityFieldModel( simulation_setup::getDefaultGravityFieldSettings(
                        "Earth", TUDAT_NAN, TUDAT_NAN ), "Earth", bodyMap ) );

    bodyMap["Venus"]->setGravityFieldModel( simulation_setup::createGravityFieldModel( simulation_setup::getDefaultGravityFieldSettings(
                        "Venus", TUDAT_NAN, TUDAT_NAN ), "Venus", bodyMap ) );

    bodyMap["Jupiter"]->setGravityFieldModel( simulation_setup::createGravityFieldModel( simulation_setup::getDefaultGravityFieldSettings(
                        "Jupiter", TUDAT_NAN, TUDAT_NAN ), "Jupiter", bodyMap ) );

    bodyMap["Saturn"]->setGravityFieldModel( simulation_setup::createGravityFieldModel( simulation_setup::getDefaultGravityFieldSettings(
                        "Saturn", TUDAT_NAN, TUDAT_NAN ), "Saturn", bodyMap ) );


    bodyMap[ bodyToPropagate[0] ] = std::make_shared< simulation_setup::Body >( );
    bodyMap[ bodyToPropagate[0] ]->setEphemeris( std::make_shared< ephemerides::TabulatedCartesianEphemeris< > >(
                    std::shared_ptr< interpolators::OneDimensionalInterpolator
                    < double, Eigen::Vector6d > >( ), frameOrigin, frameOrientation ) );

    setGlobalFrameBodyEphemerides( bodyMap, frameOrigin, frameOrientation );

    // create acceleration map
    basic_astrodynamics::AccelerationMap accelerationMap = propagators::setupAccelerationMapLambertTargeter(centralBody[0],
                                                                                               bodyToPropagate[0], bodyMap);


    // Create variable vector.
    Eigen::VectorXd variableVector( numberOfLegs + 1 );
    variableVector << -789.8117, 158.302027105278, 449.385873819743, 54.7489684339665,
            1024.36205846918, 4552.30796805542, 1;
    variableVector *= physical_constants::JULIAN_DAY;

    // Create departure and capture variables.
    Eigen::VectorXd semiMajorAxes( 2 ), eccentricities( 2 );
    semiMajorAxes << std::numeric_limits< double >::infinity( ), 1.0895e8 / 0.02;
    eccentricities << 0.0, 0.98;


    // Create minimum pericenter radii vector
    Eigen::VectorXd minimumPericenterRadii( numberOfLegs );
    minimumPericenterRadii << 6778000.0, 6351800.0, 6351800.0, 6778000.0, 600000000.0, 600000000.0;



    std::map< int, std::map< double, Eigen::Vector6d > > lambertTargeterResultForEachLeg;
    std::map< int, std::map< double, Eigen::Vector6d > > fullProblemResultForEachLeg;

    std::map< int, std::pair< Eigen::Vector6d, Eigen::Vector6d > > differenceStateArrivalAndDeparturePerLeg =
            propagators::getDifferenceFullPropagationWrtLambertTargeterMGA( bodyMap, accelerationMap, numberOfLegs, nameBodiesTrajectory,
                            nameBodiesTrajectory, centralBody, bodyToPropagate, legTypeVector, variableVector, minimumPericenterRadii,
                            semiMajorAxes, eccentricities, integratorSettings);

    for( std::map< int, std::pair< Eigen::Vector6d, Eigen::Vector6d > >::iterator
         itr = differenceStateArrivalAndDeparturePerLeg.begin( );
            itr != differenceStateArrivalAndDeparturePerLeg.end( ); itr++ ){

        std::cout << "Departure body: " << nameBodiesTrajectory[itr->first] << "\n\n";
        std::cout << "Arrival body: " << nameBodiesTrajectory[itr->first + 1] << "\n\n";
        std::cout << "state difference departure: " << differenceStateArrivalAndDeparturePerLeg[itr->first].first << "\n\n";
        std::cout << "state difference arrival: " << differenceStateArrivalAndDeparturePerLeg[itr->first].second << "\n\n";

        for( int i = 0; i < 3; i++ )
        {
            BOOST_CHECK_SMALL( std::fabs( differenceStateArrivalAndDeparturePerLeg[itr->first].first( i ) ), 1.0 );
            BOOST_CHECK_SMALL( std::fabs( differenceStateArrivalAndDeparturePerLeg[itr->first].first( i + 3 ) ), 1.0E-6 );
            BOOST_CHECK_SMALL( std::fabs( differenceStateArrivalAndDeparturePerLeg[itr->first].second( i ) ), 1.0 );
            BOOST_CHECK_SMALL( std::fabs( differenceStateArrivalAndDeparturePerLeg[itr->first].second( i + 3 ) ), 1.0E-6 );
        }

    }

}


//! Test of the full propagation of a MGA trajectory including deep-space manoeuvres
BOOST_AUTO_TEST_CASE( testFullPropagationMGAwithDSM )
{

    std::cout << "Messenger trajectory: " << "\n\n";

    // Specify required parameters
    // Specify the number of legs and type of legs.
    int numberOfLegs = 5;
    std::vector< transfer_trajectories::TransferLegType > legTypeVector;
    legTypeVector.resize( numberOfLegs );
    legTypeVector[ 0 ] = transfer_trajectories::mga1DsmVelocity_Departure;
    legTypeVector[ 1 ] = transfer_trajectories::mga1DsmVelocity_Swingby;
    legTypeVector[ 2 ] = transfer_trajectories::mga1DsmVelocity_Swingby;
    legTypeVector[ 3 ] = transfer_trajectories::mga1DsmVelocity_Swingby;
    legTypeVector[ 4 ] = transfer_trajectories::capture;

    // Name of the bodies involved in the trajectory
    std::vector< std::string > nameBodiesAndManoeuvresTrajectory;
    nameBodiesAndManoeuvresTrajectory.push_back("Earth");
    nameBodiesAndManoeuvresTrajectory.push_back("DSM1");
    nameBodiesAndManoeuvresTrajectory.push_back("Earth");
    nameBodiesAndManoeuvresTrajectory.push_back("DSM2");
    nameBodiesAndManoeuvresTrajectory.push_back("Venus");
    nameBodiesAndManoeuvresTrajectory.push_back("DSM3");
    nameBodiesAndManoeuvresTrajectory.push_back("Venus");
    nameBodiesAndManoeuvresTrajectory.push_back("DSM4");
    nameBodiesAndManoeuvresTrajectory.push_back("Mercury");

    std::vector< std::string > transferBodyTrajectory;
    transferBodyTrajectory.push_back("Earth");
    transferBodyTrajectory.push_back("Earth");
    transferBodyTrajectory.push_back("Venus");
    transferBodyTrajectory.push_back("Venus");
    transferBodyTrajectory.push_back("Mercury");



    std::vector< std::string > centralBody;
    centralBody.push_back( "Sun" );
    std::vector< std::string > bodyToPropagate;
    bodyToPropagate.push_back( "spacecraft" );

    spice_interface::loadStandardSpiceKernels( );

    std::map< std::string, std::shared_ptr< simulation_setup::BodySettings > > bodySettings =
                    simulation_setup::getDefaultBodySettings( centralBody );

    // Define central body ephemeris settings.
    std::string frameOrigin = "SSB";
    std::string frameOrientation = "J2000";
    bodySettings[ centralBody[0] ]->ephemerisSettings = std::make_shared< simulation_setup::ConstantEphemerisSettings >(
            ( Eigen::Vector6d( ) << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ).finished( ), frameOrigin, frameOrientation );

    bodySettings[ centralBody[0] ]->ephemerisSettings->resetFrameOrientation( frameOrientation );
    bodySettings[ centralBody[0] ]->rotationModelSettings->resetOriginalFrame( frameOrientation );


    // Create body map.
    simulation_setup::NamedBodyMap bodyMap = createBodies( bodySettings );

    // ephemeris
    bodyMap["Earth"] = std::make_shared< simulation_setup::Body >( );
    bodyMap["Earth"]->setEphemeris( std::make_shared< ephemerides::ApproximatePlanetPositions >(
                                        ephemerides::ApproximatePlanetPositionsBase::BodiesWithEphemerisData::earthMoonBarycenter ));
    bodyMap["Venus"] = std::make_shared< simulation_setup::Body >( );
    bodyMap["Venus"]->setEphemeris( std::make_shared< ephemerides::ApproximatePlanetPositions >(
                                        ephemerides::ApproximatePlanetPositionsBase::BodiesWithEphemerisData::venus ));
    bodyMap["Mercury"] = std::make_shared< simulation_setup::Body >( );
    bodyMap["Mercury"]->setEphemeris( std::make_shared< ephemerides::ApproximatePlanetPositions >(
                                        ephemerides::ApproximatePlanetPositionsBase::BodiesWithEphemerisData::mercury ));


    // gravity field
    bodyMap["Earth"]->setGravityFieldModel( simulation_setup::createGravityFieldModel( simulation_setup::getDefaultGravityFieldSettings(
                        "Earth", TUDAT_NAN, TUDAT_NAN ), "Earth", bodyMap ) );

    bodyMap["Venus"]->setGravityFieldModel( simulation_setup::createGravityFieldModel( simulation_setup::getDefaultGravityFieldSettings(
                        "Venus", TUDAT_NAN, TUDAT_NAN ), "Venus", bodyMap ) );

    bodyMap["Mercury"]->setGravityFieldModel( simulation_setup::createGravityFieldModel( simulation_setup::getDefaultGravityFieldSettings(
                        "Mercury", TUDAT_NAN, TUDAT_NAN ), "Mercury", bodyMap ) );


    bodyMap[ bodyToPropagate[0] ] = std::make_shared< simulation_setup::Body >( );
    bodyMap[ bodyToPropagate[0] ]->setEphemeris( std::make_shared< ephemerides::TabulatedCartesianEphemeris< > >(
                    std::shared_ptr< interpolators::OneDimensionalInterpolator
                    < double, Eigen::Vector6d > >( ), frameOrigin, frameOrientation ) );

    setGlobalFrameBodyEphemerides( bodyMap, frameOrigin, frameOrientation );



    // create acceleration map
    basic_astrodynamics::AccelerationMap accelerationMap = propagators::setupAccelerationMapLambertTargeter(centralBody[0],
                                                                                               bodyToPropagate[0], bodyMap);

    // Create variable vector.
    Eigen::VectorXd variableVector;
    variableVector.resize( numberOfLegs /*time of flight*/ + 1 /*start epoch*/ +
                           4 * ( numberOfLegs - 1 ) /*additional variables for model, except the final capture leg*/ );

    // Add the time of flight and start epoch, which are in JD.
    variableVector << 1171.64503236 * physical_constants::JULIAN_DAY,
            399.999999715 * physical_constants::JULIAN_DAY,
            178.372255301 * physical_constants::JULIAN_DAY,
            299.223139512 * physical_constants::JULIAN_DAY,
            180.510754824 * physical_constants::JULIAN_DAY,
            1, // The capture time is irrelevant for the final leg.
            // Add the additional variables.
            0.234594654679, 1408.99421278, 0.37992647165 * 2 * 3.14159265358979,
            std::acos(  2 * 0.498004040298 - 1. ) - 3.14159265358979 / 2, // 1st leg.
            0.0964769387134, 1.35077257078, 1.80629232251 * 6.378e6, 0.0, // 2nd leg.
            0.829948744508, 1.09554368115, 3.04129845698 * 6.052e6, 0.0, // 3rd leg.
            0.317174785637, 1.34317576594, 1.10000000891 * 6.052e6, 0.0; // 4th leg.


    // Create minimum pericenter radii vector
    Eigen::VectorXd minimumPericenterRadii( numberOfLegs );
    minimumPericenterRadii << TUDAT_NAN, TUDAT_NAN, TUDAT_NAN, TUDAT_NAN, TUDAT_NAN;

    // Create departure and capture variables.
    Eigen::VectorXd semiMajorAxes( 2 ), eccentricities( 2 );
    semiMajorAxes << std::numeric_limits< double >::infinity( ),
            std::numeric_limits< double >::infinity( );
    eccentricities << 0.0, 0.0;


    double initialTime = 0.0;
    double fixedStepSize = 1000.0;

    // Define integrator settings.
    std::shared_ptr< numerical_integrators::IntegratorSettings< double > > integratorSettings =
            std::make_shared < numerical_integrators::IntegratorSettings < > >
                ( numerical_integrators::rungeKutta4, initialTime, fixedStepSize);



    std::map< int, std::map< double, Eigen::Vector6d > > lambertTargeterResultForEachLeg;
    std::map< int, std::map< double, Eigen::Vector6d > > fullProblemResultForEachLeg;

    std::map< int, std::pair< Eigen::Vector6d, Eigen::Vector6d > > differenceStateArrivalAndDeparturePerLeg =
            propagators::getDifferenceFullPropagationWrtLambertTargeterMGA( bodyMap, accelerationMap, numberOfLegs, transferBodyTrajectory,
                               nameBodiesAndManoeuvresTrajectory, centralBody, bodyToPropagate, legTypeVector, variableVector, minimumPericenterRadii,
                               semiMajorAxes, eccentricities, integratorSettings);

    for( std::map< int, std::pair< Eigen::Vector6d, Eigen::Vector6d > >::iterator
         itr = differenceStateArrivalAndDeparturePerLeg.begin( );
            itr != differenceStateArrivalAndDeparturePerLeg.end( ); itr++ ){

        std::cout << "Departure body: " << nameBodiesAndManoeuvresTrajectory[itr->first] << "\n\n";
        std::cout << "Arrival body: " << nameBodiesAndManoeuvresTrajectory[itr->first + 1] << "\n\n";
        std::cout << "state difference departure: " << differenceStateArrivalAndDeparturePerLeg[itr->first].first << "\n\n";
        std::cout << "state difference arrival: " << differenceStateArrivalAndDeparturePerLeg[itr->first].second << "\n\n";


        for( int i = 0; i < 3; i++ )
        {
            BOOST_CHECK_SMALL( std::fabs( differenceStateArrivalAndDeparturePerLeg[itr->first].first( i ) ), 1.0 );
            BOOST_CHECK_SMALL( std::fabs( differenceStateArrivalAndDeparturePerLeg[itr->first].first( i + 3 ) ), 1.0E-6 );
            BOOST_CHECK_SMALL( std::fabs( differenceStateArrivalAndDeparturePerLeg[itr->first].second( i ) ), 1.0 );
            BOOST_CHECK_SMALL( std::fabs( differenceStateArrivalAndDeparturePerLeg[itr->first].second( i + 3 ) ), 1.0E-6 );
        }


    }

}



}

}

}


