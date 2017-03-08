/*    Copyright (c) 2010-2017, Delft University of Technology
 *    All rigths reserved
 *
 *    This file is part of the Tudat. Redistribution and use in source and
 *    binary forms, with or without modification, are permitted exclusively
 *    under the terms of the Modified BSD license. You should have received
 *    a copy of the license with this file. If not, please or visit:
 *    http://tudat.tudelft.nl/LICENSE.
 */


#ifndef CREATEOBSERVATIONMANAGER_H
#define CREATEOBSERVATIONMANAGER_H

#include <vector>
#include <map>

#include <boost/shared_ptr.hpp>

#include "Tudat/Astrodynamics/ObservationModels/observationManager.h"
#include "Tudat/SimulationSetup/EstimationSetup/createObservationModel.h"
#include "Tudat/SimulationSetup/EstimationSetup/createObservationPartials.h"
#include "Tudat/Astrodynamics/ObservationModels/oneWayRangeObservationModel.h"

namespace tudat
{

namespace observation_models
{

//! Function to create an object to simulate observations of a given type
/*!
 *  Function to create an object to simulate observations of a given type
 *  \param observableType Type of observable for which object is to simulate ObservationSimulator
 *  \param linkEnds List of LinkEnds for which the object is to be able to simulate observations of this kind
 *  \param bodyMap Map of Body objects that comprise the environment
 *  \param singleObservableCorrections Settings for light-time corrections (per link end) that are to be used.
 *  \return Object that simulates the observables according to the provided settings.
 */
template< int ObservationSize = 1, typename ObservationScalarType = double, typename TimeType = double >
boost::shared_ptr< ObservationSimulator< ObservationSize, ObservationScalarType, TimeType > > createObservationSimulator(
        const ObservableType observableType,
        const std::vector< LinkEnds > linkEnds,
        const simulation_setup::NamedBodyMap &bodyMap,
        const LightTimeCorrectionSettingsMap& singleObservableCorrections =
        LightTimeCorrectionSettingsMap( ) )
{
    std::map< LinkEnds, boost::shared_ptr< ObservationModel< ObservationSize, ObservationScalarType, TimeType > > >
            observationModels;

    // Iterate over all link ends
    for( unsigned int i = 0; i < linkEnds.size( ); i++ )
    {

        // Create single observation model.
        std::vector< boost::shared_ptr< LightTimeCorrectionSettings > > currentLightTimeCorrections;
        if( singleObservableCorrections.count( linkEnds.at( i ) ) > 0 )
        {
            currentLightTimeCorrections = singleObservableCorrections.at( linkEnds.at( i ) );
        }
        observationModels[ linkEnds[ i ] ] = ObservationModelCreator<
                ObservationSize, ObservationScalarType, TimeType >::createObservationModel(
                    observableType, linkEnds.at( i ), bodyMap, currentLightTimeCorrections );

    }

    return boost::make_shared< ObservationSimulator< ObservationSize, ObservationScalarType, TimeType > >(
                observableType, observationModels );


}

//! Function to create an object to simulate observations of a given type and associated partials
/*!
 *  Function to create an object to simulate observations of a given type and associated partials
 *  \param observableType Type of observable for which object is to simulate ObservationSimulator
 *  \param linkEnds List of LinkEnds for which the object is to be able to simulate observations of this kind
 *  \param bodyMap Map of Body objects that comprise the environment
 *  \param parametersToEstimate Object containing the list of all parameters that are to be estimated
 *  \param stateTransitionMatrixInterface Object used to compute the state transition/sensitivity matrix at a given time
 *  \param bodyMap Map of Body objects that comprise the environment
 *  \param singleObservableCorrections Settings for light-time corrections (per link end) that are to be used.
 *  \return Object that simulates the observations of a given type and associated partials
 */
template< int ObservationSize = 1, typename ObservationScalarType = double, typename TimeType = double >
boost::shared_ptr< ObservationManagerBase< ObservationScalarType, TimeType > > createObservationManager(
        const ObservableType observableType,
        const std::vector< LinkEnds > linkEnds,
        const simulation_setup::NamedBodyMap &bodyMap,
        const boost::shared_ptr< estimatable_parameters::EstimatableParameterSet< ObservationScalarType > >
        parametersToEstimate,
        const boost::shared_ptr< propagators::CombinedStateTransitionAndSensitivityMatrixInterface >
        stateTransitionMatrixInterface,
        const LightTimeCorrectionSettingsMap& singleObservableCorrections =
        LightTimeCorrectionSettingsMap( ) )
{
    using namespace observation_models;
    using namespace observation_partials;

    boost::shared_ptr< ObservationManagerBase< ObservationScalarType, TimeType > > observationManager;

    // Create observation simulator
    boost::shared_ptr< ObservationSimulator< ObservationSize, ObservationScalarType, TimeType > > observationSimulator =
            createObservationSimulator< ObservationSize, ObservationScalarType, TimeType >(
                observableType, linkEnds, bodyMap, singleObservableCorrections );

    // Get light-time corrections for current observable
    PerLinkEndPerLightTimeSolutionCorrections lightTimeCorrectionList =
            getLightTimeCorrectionsList( observationSimulator->getObservationModels( ) );

    // Create observation partials for all link ends/parameters
    boost::shared_ptr< ObservationPartialCreator< ObservationSize, ObservationScalarType > > observationPartialCreator;
    std::map< LinkEnds, std::pair< std::map< std::pair< int, int >,
            boost::shared_ptr< ObservationPartial< ObservationSize > > >,
            boost::shared_ptr< PositionPartialScaling > > > observationPartialsAndScaler;
    if( parametersToEstimate != NULL )
    {
        observationPartialsAndScaler =
            observationPartialCreator->createObservationPartials(
                observableType, linkEnds, bodyMap, parametersToEstimate,
                lightTimeCorrectionList );
    }

    // Split position partial scaling and observation partial objects.
    std::map< LinkEnds, std::map< std::pair< int, int >,
            boost::shared_ptr< observation_partials::ObservationPartial< ObservationSize > > > > observationPartials;
    std::map< LinkEnds, boost::shared_ptr< observation_partials::PositionPartialScaling  > > observationPartialScalers;
    splitObservationPartialsAndScalers( observationPartialsAndScaler, observationPartials, observationPartialScalers );


    return boost::make_shared< ObservationManager< ObservationSize, ObservationScalarType, TimeType > >(
                observableType, observationSimulator, observationPartials,
                observationPartialScalers, stateTransitionMatrixInterface );
}


//! Function to create an object to simulate observations of a given type and associated partials
/*!
 *  Function to create an object to simulate observations of a given type and associated partials
 *  \param observableType Type of observable for which object is to simulate ObservationSimulator
 *  \param linkEnds List of LinkEnds for which the object is to be able to simulate observations of this kind
 *  \param bodyMap Map of Body objects that comprise the environment
 *  \param parametersToEstimate Object containing the list of all parameters that are to be estimated
 *  \param stateTransitionMatrixInterface Object used to compute the state transition/sensitivity matrix at a given time
 *  \param bodyMap Map of Body objects that comprise the environment
 *  \param singleObservableCorrections Settings for light-time corrections (per link end) that are to be used.
 *  \return Object that simulates the observations of a given type and associated partials
 */
template< typename ObservationScalarType = double, typename TimeType = double >
boost::shared_ptr< ObservationManagerBase< ObservationScalarType, TimeType > > createObservationManagerBase(
        const ObservableType observableType,
        const std::vector< LinkEnds > linkEnds,
        const simulation_setup::NamedBodyMap &bodyMap,
        const boost::shared_ptr< estimatable_parameters::EstimatableParameterSet< ObservationScalarType > > parametersToEstimate,
        const boost::shared_ptr< propagators::CombinedStateTransitionAndSensitivityMatrixInterface > stateTransitionMatrixInterface,
        const LightTimeCorrectionSettingsMap& singleObservableCorrections =
        LightTimeCorrectionSettingsMap( ) )
{
    boost::shared_ptr< ObservationManagerBase< ObservationScalarType, TimeType > > observationManager;
    switch( observableType )
    {
    case oneWayRange:
        observationManager = createObservationManager< 1, ObservationScalarType, TimeType >(
                    observableType, linkEnds, bodyMap, parametersToEstimate,
                    stateTransitionMatrixInterface, singleObservableCorrections );
        break;
    case oneWayDoppler:
        observationManager = createObservationManager< 1, ObservationScalarType, TimeType >(
                    observableType, linkEnds, bodyMap, parametersToEstimate,
                    stateTransitionMatrixInterface, singleObservableCorrections );
        break;
    case angular_position:
        observationManager = createObservationManager< 2, ObservationScalarType, TimeType >(
                    observableType, linkEnds, bodyMap, parametersToEstimate,
                    stateTransitionMatrixInterface, singleObservableCorrections );
        break;
    case position_observable:
        observationManager = createObservationManager< 3, ObservationScalarType, TimeType >(
                    observableType, linkEnds, bodyMap, parametersToEstimate,
                    stateTransitionMatrixInterface, singleObservableCorrections );
        break;
    default:
        throw std::runtime_error(
                    "Error when making observation manager, could not identify observable type " +
                    boost::lexical_cast< std::string >( observableType ) );
    }
    return observationManager;
}

}


}


#endif // CREATEOBSERVATIONMANAGER_H
