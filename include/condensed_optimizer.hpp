#pragma once

#include <g2o/core/sparse_block_matrix.h>
#include <cameraManager.hpp>


namespace visual_slam{

    struct local_map{

        typedef std::shared_ptr<local_map> Ptr;
        typedef std::shared_ptr<const local_map> ConstPtr;

        int seq;
        Camera::Ptr origin_ptr;
        Camera::Ptr first_frame_ptr; // if not the first local map this frame is a separator  
        Camera::Ptr last_frame_ptr; // this frame is a separator
        g2o::SparseBlockMatrix< Eigen::MatrixXd > marginals;
        std::vector<unsigned long> observed_landmarks_ids;
        std::list<MapPoint::Ptr> separators_mappoints; // map points whose observations are shared with the previous and successive map        

    };    

    class condensed_optimizer{

        public:

        condensed_optimizer( std::shared_ptr<const std::vector<Camera::Ptr>> vector_ptr, world_Map::ConstPtr ptr_to_map){
            frames_vector_ptr_ = vector_ptr;
            landmarks_map_ = ptr_to_map;
        }

        void write_observed_landmarks(local_map::Ptr map_ptr); // call this method when writing the pointer to the last frame: iterate over the frames that are part of the map 
                                            // and insert the ids of all the observed landmarks.

        void local_maps_manager(); 
        // Create a new local map OR insert a new frame into the latest local map OR add reference to origin.
        // Call after every keyframe insertion.

        void write_separators();
        // Call when creating a new local map.
        // It writes the separators of the second last local map (if size of maps_ is at least 3)

        // void optimize_local_map(){};
        // // optimize local map with projection measurements and compute marginals.
        // // Call after the pointer to the last frame has been written.

    

        private:

        std::shared_ptr<const std::vector<Camera::Ptr>> frames_vector_ptr_;
        std::vector<local_map::Ptr> maps_;
        world_Map::ConstPtr landmarks_map_;

    };



} // namespace visual_slam