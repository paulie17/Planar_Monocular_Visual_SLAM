#include <Map.hpp>

namespace visual_slam{

    bool world_Map::insertMapPoint ( MapPoint::Ptr map_point ) // Return true if insertion is successful, false otherwise
    {
        if ( map_points.find(map_point->id) == map_points.end() )
        {
            map_points.insert( std::make_pair(map_point->id, map_point) );
            return true;
        }
        else 
        {
            return false;
        }
    }


}