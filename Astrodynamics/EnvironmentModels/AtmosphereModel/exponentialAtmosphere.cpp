/*! \file exponentialAtmosphere.cpp
 *    Source file that defines the exponential atmosphere model included in Tudat.
 *
 *    Path              : /Astrodynamics/EnvironmentModels/AtmosphereModel/
 *    Version           : 4
 *    Check status      : Checked
 *
 *    Author            : F.M. Engelen
 *    Affiliation       : Delft University of Technology
 *    E-mail address    : F.M.Engelen@student.tudelft.nl
 *
 *    Checker           : J.C.P Melman
 *    Affiliation       : Delft University of Technology
 *    E-mail address    : J.C.P.Melman@tudelft.nl
 *
 *    Date created      : 24 February, 2011
 *    Last modified     : 29 June, 2011
 *
 *    References
 *
 *    Notes
 *      The accuracy of this model could be increased by implementing different
 *      values for the scale height and temperature for different altitudes
 *      (e.g., lower, middle and upper atmosphere).
 *
 *    Copyright (c) 2010-2011 Delft University of Technology.
 *
 *    This software is protected by national and international copyright.
 *    Any unauthorized use, reproduction or modification is unlawful and
 *    will be prosecuted. Commercial and non-private application of the
 *    software in any form is strictly prohibited unless otherwise granted
 *    by the authors.
 *
 *    The code is provided without any warranty; without even the implied
 *    warranty of merchantibility or fitness for a particular purpose.
 *
 *    Changelog
 *      YYMMDD    Author            Comment
 *      110224    F.M. Engelen      File created.
 *      110324    J. Melman         Added overloaded get functions.
 *      110427    F.M. Engelen      Changed input parameter to altitude, longitude and latitude.
 *      110629    F.M. Engelen      Added predefined feature.
 *      110705    F.M. Engelen      Changed to passing by reference. Changed reference values.
 */

// Include statements.
#include "Astrodynamics/EnvironmentModels/AtmosphereModel/exponentialAtmosphere.h"
#include "Astrodynamics/physicalConstants.h"
#include <cmath>

//! Set predefined exponential atmosphere settings.
void ExponentialAtmosphere::setPredefinedExponentialAtmosphere(
        BodiesWithPredefinedExponentialAtmospheres bodyWithPredefindExponentialAtmosphere )
{
    switch( bodyWithPredefindExponentialAtmosphere )
    {
    case earth:
        // Set local variables for Earth exponential atmosphere. Based on  lecture notes
        // Rocket Motion by Prof. Ir. B.A.C. Ambrosius, November 2009.

        // Set scale height.
        setScaleHeight( 7.200e3 );

        //Set density at zero altitude.
        setDensityAtZeroAltitude( 1.225 );

        //Set atmosphere temperature.
        setConstantTemperature( 246.0 );

        //Set specific gas constant.
        setSpecificGasConstant( PhysicalConstants::SPECIFIC_GAS_CONSTANT_AIR );

        break;

    default:
        std::cerr << "This is not a body with a predefined exponential atmophere." << std::endl;
    }
}

//! Get local density in the general way.
double ExponentialAtmosphere::getDensity( const double& altitude,
                                          const double& longitude,
                                          const double& latitude,
                                          const double& time )
{
    return densityAtZeroAltitude_ * exp( - altitude / scaleHeight_ );
}

//! Get local pressure in the general way.
double ExponentialAtmosphere::getPressure( const double& altitude,
                                           const double& longitude,
                                           const double& latitude,
                                           const double& time )
{
    return getDensity( altitude ) * specificGasConstant_ *
            constantTemperature_;
}

//! Get local temperature in the general way.
double ExponentialAtmosphere::getTemperature( const double& altitude,
                                              const double& longitude,
                                              const double& latitude,
                                              const double& time )
{
    return constantTemperature_;
}

// End of file.
