#pragma once

#include <MapPoint.hpp>

namespace visual_slam{

    struct world_Map{


        typedef std::shared_ptr<world_Map> Ptr;
        typedef std::shared_ptr<const world_Map> ConstPtr;

        bool insertMapPoint( MapPoint::Ptr point );


        std::unordered_map<unsigned long, MapPoint::Ptr >  map_points;        // all landmarks found
        

    }; // struct Map


} // namespace visual_slam