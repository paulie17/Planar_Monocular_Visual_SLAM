#include <condensed_optimizer.hpp>

namespace visual_slam{

    void condensed_optimizer::write_observed_landmarks(local_map::Ptr map_ptr){

        for (int i = map_ptr->first_frame_ptr->seq; i < map_ptr->last_frame_ptr->seq + 1; i++){
            
            for (auto& observations: frames_vector_ptr_->at(i)->observed_points){
                map_ptr->observed_landmarks_ids.push_back(observations.second->id);                
            }

        }

        std::sort( map_ptr->observed_landmarks_ids.begin(), map_ptr->observed_landmarks_ids.end() );
        map_ptr->observed_landmarks_ids.erase( unique( map_ptr->observed_landmarks_ids.begin(), map_ptr->observed_landmarks_ids.end() ), 
                                                        map_ptr->observed_landmarks_ids.end() );

    }

    void condensed_optimizer::local_maps_manager(){

        if(maps_.empty()){ 
            // if the map is empty create the first local map object.
            // the first local map will have the first frame ptr equal to the origin ptr
            local_map::Ptr map( new local_map());
            map->first_frame_ptr = frames_vector_ptr_->back();
            map->origin_ptr = map->first_frame_ptr;
            map->seq = maps_.size();
            maps_.push_back(map);
            return;
        }

        else if(frames_vector_ptr_->back()->seq % 9 == 0 && maps_.back()->last_frame_ptr==nullptr){
            // Every 10 frames write the 10th frame as the last of the current local map and the first of the new local map
            
            maps_.back()->last_frame_ptr = frames_vector_ptr_->back();
            write_observed_landmarks(maps_.back());
            
            local_map::Ptr map( new local_map());
            map->first_frame_ptr = frames_vector_ptr_->back();
            map->seq = maps_.size();
            maps_.push_back(map);

            return;
        }

        
        else if(frames_vector_ptr_->back()->seq - maps_.back()->first_frame_ptr->seq == 4 && maps_.back()->seq != 0){
            // if the new local map is not the first one to be written, save the origin when you reveive the 5th frame 
            maps_.back()->origin_ptr = frames_vector_ptr_->back();
            return;
        }
    }


    void condensed_optimizer::write_separators(){
            
        if (maps_.size() < 3){
            return;
        }

        int index = maps_.size() - 2;
        local_map::Ptr map_to_be_written = maps_[index];
        local_map::Ptr previous_one = maps_[index-1];
        local_map::Ptr next_one = maps_[index+1];

        std::vector<long unsigned> previous_intersection;
        std::vector<long unsigned> final_intersection;

        std::set_intersection(map_to_be_written->observed_landmarks_ids.begin(), map_to_be_written->observed_landmarks_ids.end(), 
                                previous_one->observed_landmarks_ids.begin(), previous_one->observed_landmarks_ids.end(), 
                                previous_intersection.begin());

        std::set_intersection(next_one->observed_landmarks_ids.begin(), next_one->observed_landmarks_ids.end(), 
                                previous_intersection.begin(), previous_intersection.end(), 
                                final_intersection.begin());

        for ( int i = 0; i < final_intersection.size(); i ++){

            map_to_be_written->separators_mappoints.push_back(landmarks_map_->map_points.at(final_intersection[i]));

        }

    }

    // void condensed_optimizer::optimize_local_map(){


    // }


}