/*    Copyright (c) 2010-2022, Delft University of Technology
 *    All rigths reserved
 *
 *    This file is part of the Tudat. Redistribution and use in source and
 *    binary forms, with or without modification, are permitted exclusively
 *    under the terms of the Modified BSD license. You should have received
 *    a copy of the license with this file. If not, please or visit:
 *    http://tudat.tudelft.nl/LICENSE.
 *
 *    References
 *      Ganeff, M.I. Solar radiation pressure benchmark data script,
 *          solarRadiationPressureBenchmarkData.m, available at http://tudat.tudelft.nl, 2012.
 *
 */

#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MAIN

#include <iostream>

#include <boost/test/unit_test.hpp>

#include "tudat/basics/testMacros.h"
#include "tudat/astro/electromagnetism/radiationPressureTargetModel.h"
#include "tudat/astro/electromagnetism/radiationSourceModel.h"
#include "tudat/math/basic/coordinateConversions.h"


namespace tudat
{
namespace unit_tests
{

using namespace tudat::electromagnetism;
using TargetPanel = PaneledRadiationPressureTargetModel::Panel;

BOOST_AUTO_TEST_SUITE(test_radiation_pressure_target_model)

//! Check force due to solar constant based on data from (Montenbruck, 2000)
BOOST_AUTO_TEST_CASE( testCannonballRadiationPressureTargetModelAtAU )
{
    // From (Montenbruck, 2000) Eq. (3.69)
    const auto expectedForce = Eigen::Vector3d(4.56e-6, 0, 0);

    CannonballRadiationPressureTargetModel targetModel(1, 1);

    const auto actualForce = targetModel.evaluateRadiationPressureForce(
            1367,
            Eigen::Vector3d::UnitX());

    TUDAT_CHECK_MATRIX_CLOSE_FRACTION(actualForce, expectedForce, 0.01);
}

//! Check force with benchmark data is obtained from MATLAB script (Ganeff, 2012)
BOOST_AUTO_TEST_CASE( testCannonballRadiationPressureTargetModelGaneff )
{
    const auto expectedForce = Eigen::Vector3d( -0.975383093968723e-6, -0.975383093968723e-6, 0.0 );

    CannonballRadiationPressureTargetModel targetModel(0.5, 1.21);

    const auto sourceIrradiance = 683.5;  // at (1 AU, 1 AU, 0) from Sun
    const auto sourceToTargetDirection = Eigen::Vector3d(-1, -1, 0).normalized();
    const auto actualForce = targetModel.evaluateRadiationPressureForce(
            sourceIrradiance,
            sourceToTargetDirection);

    TUDAT_CHECK_MATRIX_CLOSE_FRACTION(actualForce, expectedForce, 1e-3);
}

//! Check force when radiation pressure coefficient is zero
BOOST_AUTO_TEST_CASE( testCannonballRadiationPressureTargetModelZeroCoeff )
{
    const auto expectedForce = Eigen::Vector3d::Zero();

    CannonballRadiationPressureTargetModel targetModel(1, 0);

    const auto sourceIrradiance = 1367;
    const auto sourceToTargetDirection = Eigen::Vector3d(-1, -1, 0).normalized();
    const auto actualForce = targetModel.evaluateRadiationPressureForce(
            sourceIrradiance,
            sourceToTargetDirection);

    TUDAT_CHECK_MATRIX_CLOSE_FRACTION(actualForce, expectedForce, 1e-15);
}

//! Check if force is zero if all panels point away from incident radiation
BOOST_AUTO_TEST_CASE( testPaneledRadiationPressureTargetModel_PointingAway )
{
    const auto expectedForce = Eigen::Vector3d::Zero();

    const auto reflectionLaw = std::make_shared<SpecularDiffuseMixReflectionLaw>(0.2, 0.4, 0.4);
    PaneledRadiationPressureTargetModel targetModel({
            TargetPanel(1, Eigen::Vector3d(1, 0, -1).normalized(), reflectionLaw),
            TargetPanel(1, Eigen::Vector3d(-1, 0, -2).normalized(), reflectionLaw),
            TargetPanel(1, Eigen::Vector3d(0, 1, -3).normalized(), reflectionLaw),
            TargetPanel(1, Eigen::Vector3d(0, -1, -4).normalized(), reflectionLaw)
    });
    targetModel.updateMembers(TUDAT_NAN);

    const auto sourceIrradiance = 1000;
    const auto sourceToTargetDirection = Eigen::Vector3d(0, 0, -1).normalized();
    const auto actualForce = targetModel.evaluateRadiationPressureForce(
            sourceIrradiance,
            sourceToTargetDirection);

    TUDAT_CHECK_MATRIX_CLOSE_FRACTION(actualForce, expectedForce, 1e-15);
}

//! Check if lateral forces due to angled panels cancel and all force is along z-axis
BOOST_AUTO_TEST_CASE( testPaneledRadiationPressureTargetModel_LateralCancellation )
{
    // Purely diffuse Lambertian radiation so radiation pressure force is along normal of each panel
    const auto reflectionLaw = std::make_shared<SpecularDiffuseMixReflectionLaw>(0, 0, 1);
    PaneledRadiationPressureTargetModel targetModel({
            TargetPanel(1, Eigen::Vector3d(1, 0, 1).normalized(), reflectionLaw),
            TargetPanel(1, Eigen::Vector3d(-1, 0, 1).normalized(), reflectionLaw),
            TargetPanel(1, Eigen::Vector3d(0, 1, 1).normalized(), reflectionLaw),
            TargetPanel(1, Eigen::Vector3d(0, -1, 1).normalized(), reflectionLaw)
    });
    targetModel.updateMembers(TUDAT_NAN);

    const auto sourceIrradiance = 1000;
    const auto sourceToTargetDirection = Eigen::Vector3d(0, 0, -1).normalized();
    const auto actualForce = targetModel.evaluateRadiationPressureForce(
            sourceIrradiance,
            sourceToTargetDirection);

    // No lateral force
    BOOST_CHECK_CLOSE(actualForce[0], 0, 1e-15);
    BOOST_CHECK_CLOSE(actualForce[1], 0, 1e-15);
    // Only -z force due to -z incident radiation
    BOOST_CHECK_LE(actualForce[2], 0);
}

//! Check if cannonball and paneled model agree for a single equivalent panel
BOOST_AUTO_TEST_CASE( testRadiationPressureTargetModel_EquivalentSinglePanel )
{
    const auto radius = 4.2;
    const auto coefficient = 1.6;

    const double sourceIrradiance = 1000;
    const Eigen::Vector3d sourceToTargetDirection = Eigen::Vector3d(4, 3, -1).normalized();

    auto cannonballModel = CannonballRadiationPressureTargetModel(mathematical_constants::PI * radius * radius, coefficient);
    auto paneledModel = PaneledRadiationPressureTargetModel({
        TargetPanel(
                mathematical_constants::PI * radius * radius,
                -sourceToTargetDirection,
                // This gives an equivalent coefficient of 1.6
                reflectionLawFromSpecularAndDiffuseReflectivity(0.6, 0))
    });

    cannonballModel.updateMembers(TUDAT_NAN);
    paneledModel.updateMembers(TUDAT_NAN);

    const auto cannonballForce = cannonballModel.evaluateRadiationPressureForce(sourceIrradiance, sourceToTargetDirection);
    const auto paneledForce = paneledModel.evaluateRadiationPressureForce(sourceIrradiance, sourceToTargetDirection);

    TUDAT_CHECK_MATRIX_CLOSE(cannonballForce, paneledForce, 1e-10);
}

//! Check if cannonball and paneled model agree for an equivalent paneled sphere
BOOST_AUTO_TEST_CASE( testRadiationPressureTargetModel_EquivalentSphere )
{
    const auto radius = 4.2;
    // This gives approximately the equivalent force
    const auto coefficient = 1.4444446;

    const double sourceIrradiance = 1000;
    const Eigen::Vector3d sourceToTargetDirection = Eigen::Vector3d(4, 3, -1).normalized();

    auto cannonballModel = CannonballRadiationPressureTargetModel(mathematical_constants::PI * radius * radius, coefficient);

    const int numberOfPanels = 20000;
    const auto panelArea = 4 * mathematical_constants::PI * radius * radius / numberOfPanels;
    const auto pairOfAngleVectors = generateEvenlySpacedPoints_Staggered(numberOfPanels);
    const auto polarAngles = std::get<0>(pairOfAngleVectors);
    const auto azimuthAngles = std::get<1>(pairOfAngleVectors);
    const auto reflectionLaw =
            reflectionLawFromSpecularAndDiffuseReflectivity(0, 1);

    std::vector<PaneledRadiationPressureTargetModel::Panel> panels {};

    for (unsigned int i = 0; i < numberOfPanels; ++i)
    {
        const auto polarAngle = polarAngles[i];
        const auto azimuthAngle = azimuthAngles[i];

        const Eigen::Vector3d relativeCenter = coordinate_conversions::convertSphericalToCartesian(
                Eigen::Vector3d(radius, polarAngle, azimuthAngle));
        const Eigen::Vector3d surfaceNormal = relativeCenter.normalized();

        panels.emplace_back(panelArea, surfaceNormal, reflectionLaw);
    }

    auto paneledModel = PaneledRadiationPressureTargetModel(panels);

    cannonballModel.updateMembers(TUDAT_NAN);
    paneledModel.updateMembers(TUDAT_NAN);

    const auto cannonballForce = cannonballModel.evaluateRadiationPressureForce(sourceIrradiance, sourceToTargetDirection);
    const auto paneledForce = paneledModel.evaluateRadiationPressureForce(sourceIrradiance, sourceToTargetDirection);

    // Large tolerance since equivalent coefficient is not very accurate
    TUDAT_CHECK_MATRIX_CLOSE(cannonballForce, paneledForce, 1e-3);
}

BOOST_AUTO_TEST_SUITE_END()

} // namespace unit_tests
} // namespace tudat
