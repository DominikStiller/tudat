cmake_minimum_required(VERSION 3.15.0)

#TUDAT_ADD_EXECUTABLE(application_filter
#        "src/filterExample.cpp"
#        ${Tudat_PROPAGATION_LIBRARIES}
#        )

TUDAT_ADD_EXECUTABLE(application_skydiver
        "src/skydiver.cpp"
        ${Tudat_PROPAGATION_LIBRARIES}
        )
