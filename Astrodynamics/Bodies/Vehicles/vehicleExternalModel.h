/*! \file vehicleExternalModel.cpp
 *    This file contains the definition of the VehicleExternalModel class.
 *
 *    Path              : /Astrodynamics/Bodies/Vehicles/
 *    Version           : 4
 *    Check status      : Checked
 *
 *    Author            : D. Dirkx
 *    Affiliation       : Delft University of Technology
 *    E-mail address    : D.Dirkx@student.tudelft.nl
 *
 *    Checker           : J. Melman
 *    Affiliation       : Delft University of Technology
 *    E-mail address    : J.C.P.Melman@tudelft.nl
 *
 *    Date created      : 10 September, 2010
 *    Last modified     : 11 January, 2011
 *
 *    References
 *
 *    Notes
 *
 *    Copyright (c) 2010 Delft University of Technology.
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
 *      100910    D. Dirkx          First version of file.
 *      100915    D. Dirkx          Modified comments, 80 lines rule, etc.
 *      100928    D. Dirkx          Modifications following first checking
 *                                  iteration.
 *      110112    K. Kumar          Minor comment changes.
 */

#ifndef VEHICLEEXTERNALMODELS_H
#define VEHICLEEXTERNALMODELS_H

// Include statements.
#include "geometricShape.h"

//! Vehicle external model class.
/*!
 *  Class that contains the properties of the external portion of the
 *  vehicle. Current properties that can be set are:
 *  Array of pointers to VehiclePartExternalModels, to be able to set
 *  different (types of) properties for different vehicle parts.
 */
class VehicleExternalModel
{
public:

    //! Default constructor.
    /*!
     *  Default constructor.
     */
    VehicleExternalModel( );

    //! Default destructor.
    /*!
     *  Default destructor.
     */
    ~VehicleExternalModel( );

    //! Function to set external geometry.
    /*!
    * Function to set the external vehicle geometry. Geometry object is to be
    * created externally.
    * \param vehicleGeometry pointer to shape that is to be set.
    */
    void setVehicleGeometry( GeometricShape& vehicleGeometry );

    //! Function to retrieve external geometry.
    /*!
    * Function to retrieve the external geometry.
    */
    GeometricShape* getVehicleExternalGeometry( );

    //! Overloaded ostream to print class information.
    /*!
     *  Overloaded ostream to print class information; prints whether a
     *  surface geometry has been set.
     */
    friend std::ostream& operator<<( std::ostream& stream,
                                     VehicleExternalModel& vehicleExternalModel );

private:

    //! Pointer to vehicle geometry
    /*!
     * Pointer to geometric shape that represents the vehicle's external shape.
     */
    GeometricShape* pointerToVehicleGeometry_;

    //! Flag that indicates if geometric shape has been set.
    /*!
     * Boolean that is true if a geometric shape has been set.
     */
    bool isGeometrySet_;
};

#endif // VEHICLEEXTERNALMODELS_H

//End of file.
