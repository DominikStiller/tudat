#include <iostream>

#include <Eigen/Core>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/range/adaptors.hpp>

#include <tudat/simulation/simulation.h>
#include "tudat/interface/spice/spiceEphemeris.h"

using namespace tudat;
using namespace tudat::simulation_setup;
using namespace tudat::propagators;
using namespace tudat::numerical_integrators;
using namespace tudat::orbital_element_conversions;
using namespace tudat::basic_astrodynamics;
using namespace tudat::basic_mathematics;
using namespace tudat::physical_constants;
using namespace tudat::gravitation;
using namespace tudat::numerical_integrators;


void loadLROSpiceKernels();
SystemOfBodies createSimulationBodies();
AccelerationMap createSimulationAccelerations(const SystemOfBodies&);
Eigen::VectorXd createSimulationInitialState();
SingleArcDynamicsSimulator<> createSimulator(const SystemOfBodies&, const AccelerationMap&, const Eigen::VectorXd&);
void runSimulationAndSaveResults(SingleArcDynamicsSimulator<>& dynamicsSimulator);


namespace simulation_constants
{
    const auto outputFolder = "/home/dominik/dev/tudat-bundle/output/lro/baseline";
    const auto simulationDuration = 565 * 60;  // 565 min, about 5 orbital revolutions
    const auto simulationStart = "2010 JUN 26 06:00:00";
    double simulationStartEpoch;
    double simulationEndEpoch;
    const auto printInterval = simulationDuration / 10;
    const auto minStepSize = 5.0;

    const auto globalFrameOrigin = "Moon";
    const auto globalFrameOrientation = "ECLIPJ2000";

    const std::vector< std::string > bodiesToPropagate{"LRO"};
    const std::vector< std::string > centralBodies{"Moon"};
}
using namespace simulation_constants;


int main()
{
    loadLROSpiceKernels();

    simulationStartEpoch = spice_interface::convertDateStringToEphemerisTime(simulationStart);
    simulationEndEpoch = simulationStartEpoch + simulationDuration;

    auto bodies = createSimulationBodies();
    auto accelerations = createSimulationAccelerations(bodies);
    auto initialState = createSimulationInitialState();
    auto dynamicsSimulator = createSimulator(bodies, accelerations, initialState);
    runSimulationAndSaveResults(dynamicsSimulator);

    return EXIT_SUCCESS;
}

// Loads SPICE kernels for LRO
void loadLROSpiceKernels()
{
    using namespace tudat::spice_interface;

    std::string path = "/home/dominik/dev/tudat-bundle/spice/lro/data";

    // Leap seconds
    loadSpiceKernelInTudat(path + "/lsk/naif0012.tls");
    // Planetary shapes
    loadSpiceKernelInTudat(path + "/pck/pck00010.tpc");
    // Planetary gravitational parameters
    loadSpiceKernelInTudat(path + "/pck/gm_de431.tpc");

    // LRO spacecraft bus and instrument frames
    loadSpiceKernelInTudat(path + "/fk/lro_frames_2012255_v02.tf");
    // LRO spacecraft clock
    loadSpiceKernelInTudat(path + "/sclk/lro_clkcor_2022075_v00.tsc");

    // LRO ephemeris
    for(auto& entry : boost::make_iterator_range(boost::filesystem::directory_iterator(path + "/spk"), {})) {
        if (entry.path().extension() == ".bsp")
        {
            loadSpiceKernelInTudat(entry.path().string());
        }
    }

    // LRO orientation
    for(auto& entry : boost::make_iterator_range(boost::filesystem::directory_iterator(path + "/ck"), {})) {
        if (entry.path().extension() == ".bc")
        {
            loadSpiceKernelInTudat(entry.path().string());
        }
    }
}

SystemOfBodies createSimulationBodies()
{
    // Create planets
    auto bodySettings = getDefaultBodySettings({"Sun", "Earth", "Moon"});
    for(auto const& item : bodySettings.getMap()){
        auto singleBodySettings = item.second;
        singleBodySettings->ephemerisSettings->resetFrameOrientation(globalFrameOrientation);
        singleBodySettings->rotationModelSettings->resetOriginalFrame(globalFrameOrientation);
    }
//    bodySettings.at("Sun")->radiationSourceModelSettings =
//            isotropicPointRadiationSourceModelSettings(
//                    constantLuminosityModelSettings(1e33));
    bodySettings.at("Moon")->radiationSourceModelSettings =
            staticallyPaneledRadiationSourceModelSettings({
                albedoPanelRadiosityModelSettings(0.12),
                angleBasedThermalPanelRadiosityModelSettings(100, 375, 0.95)
            }, 100);

    // Create LRO
    bodySettings.addSettings("LRO");
    bodySettings.get("LRO")->constantMass = 1208.0;
    bodySettings.get("LRO")->rotationModelSettings = spiceRotationModelSettings(globalFrameOrientation, "LRO_SC_BUS");
//    bodySettings.get("LRO")->radiationPressureTargetModelSettings =
//            cannonballRadiationPressureTargetModelSettings(15.38, 1.41);
    bodySettings.get("LRO")->radiationPressureTargetModelSettings = paneledRadiationPressureTargetModelSettings({
            TargetPanelSettings(2.82, 0.29, 0.22, Eigen::Vector3d::UnitX()),
            TargetPanelSettings(2.82, 0.39, 0.19, -Eigen::Vector3d::UnitX()),
            TargetPanelSettings(3.69, 0.32, 0.23, Eigen::Vector3d::UnitY()),
            TargetPanelSettings(3.69, 0.32, 0.18, -Eigen::Vector3d::UnitY()),
            TargetPanelSettings(5.14, 0.32, 0.18, Eigen::Vector3d::UnitZ()),
            TargetPanelSettings(5.14, 0.54, 0.15, -Eigen::Vector3d::UnitZ()),
            TargetPanelSettings(11.0, 0.05, 0.05, "Sun"),
            TargetPanelSettings(11.0, 0.05, 0.05, "Sun", false),  // not officially given
            TargetPanelSettings(1.0, 0.18, 0.28, "Earth"),
            TargetPanelSettings(1.0, 0.019, 0.0495, "Earth", false),
    });

    auto bodies = createSystemOfBodies(bodySettings);
    setGlobalFrameBodyEphemerides(bodies.getMap(), globalFrameOrigin, globalFrameOrientation);

    return bodies;
}

AccelerationMap createSimulationAccelerations(const SystemOfBodies& bodies)
{
    SelectedAccelerationMap accelerationMap {
            {"LRO", {
                {"Moon", {
                        sphericalHarmonicAcceleration(100, 100),
                        radiationPressureAcceleration("Sun")
                }},
                {"Earth", {
                        sphericalHarmonicAcceleration(50, 50)
                }},
                {"Sun", {
                        pointMassGravityAcceleration(),
                        radiationPressureAcceleration()
                }},
            }}
    };
    return createAccelerationModelsMap(bodies, accelerationMap, bodiesToPropagate, centralBodies);
}

Eigen::VectorXd createSimulationInitialState()
{
    auto ephemerisLRO = std::make_shared<ephemerides::SpiceEphemeris>(
            "LRO", globalFrameOrigin, false, false, false, globalFrameOrientation);
    auto initialState = ephemerisLRO->getCartesianState(simulationStartEpoch);
    return initialState;
}

SingleArcDynamicsSimulator<> createSimulator(
        const SystemOfBodies& bodies, const AccelerationMap& accelerations, const Eigen::VectorXd& initialState)
{
    std::vector< std::shared_ptr< SingleDependentVariableSaveSettings > > dependentVariablesList
            {
                    relativePositionDependentVariable("LRO", "Moon"),
                    relativeVelocityDependentVariable("LRO", "Moon"),
                    keplerianStateDependentVariable("LRO", "Moon"),
                    relativePositionDependentVariable("Sun", "Moon"),
                    singleAccelerationDependentVariable(spherical_harmonic_gravity, "LRO", "Moon"),
                    singleAccelerationDependentVariable(spherical_harmonic_gravity, "LRO", "Earth"),
                    singleAccelerationDependentVariable(point_mass_gravity, "LRO", "Sun"),
                    singleAccelerationDependentVariable(radiation_pressure_acceleration, "LRO", "Sun"),
                    singleAccelerationDependentVariable(radiation_pressure_acceleration, "LRO", "Moon"),
            };

    auto propagatorSettings =  translationalStatePropagatorSettings (
            centralBodies, accelerations, bodiesToPropagate, initialState, simulationEndEpoch, cowell,
            createDependentVariableSaveSettings(dependentVariablesList), printInterval);

    auto integratorSettings = adamsBashforthMoultonSettings(
                    simulationStartEpoch, minStepSize, minStepSize, minStepSize);

    SingleArcDynamicsSimulator< > dynamicsSimulator(
            bodies, integratorSettings, propagatorSettings, true, false, false );
    return dynamicsSimulator;
}

void runSimulationAndSaveResults(SingleArcDynamicsSimulator<>& dynamicsSimulator)
{
    auto integrationResult = dynamicsSimulator.getEquationsOfMotionNumericalSolution();
    auto dependentVariableResult = dynamicsSimulator.getDependentVariableHistory();

    // Write perturbed satellite propagation history to file.
    input_output::writeDataMapToTextFile( integrationResult,
                                          "lro_propagation_history.dat",
                                          outputFolder,
                                          "",
                                          std::numeric_limits< double >::digits10,
                                          std::numeric_limits< double >::digits10,
                                          ",");

    input_output::writeDataMapToTextFile( dependentVariableResult,
                                          "lro_dependent_variable_history.dat",
                                          outputFolder,
                                          "",
                                          std::numeric_limits< double >::digits10,
                                          std::numeric_limits< double >::digits10,
                                          ",");
}
