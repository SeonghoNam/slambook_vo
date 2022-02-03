add_library( myslam SHARED
    Frame.cpp
    MapPoint.cpp
    Map.cpp
    Camera.cpp
    Config.cpp
)

target_link_libraries( myslam
    ${THIRD_PARTY_LIBS}
)