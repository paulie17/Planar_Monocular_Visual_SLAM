#include <condensed_optimizer.hpp>

namespace visual_slam{

    void condensed_optimizer::write_observed_landmarks(local_map& lmap){

        for (int i = lmap.first_frame_ptr->seq; i < lmap.last_frame_ptr->seq + 1; i++){
            
            for (auto& observations: frames_vector_ptr_->at(i)->observed_points){
                lmap.observed_landmarks_ids.push_back(observations.second->id);                
            }

        }

        std::sort( lmap.observed_landmarks_ids.begin(), lmap.observed_landmarks_ids.end() );
        lmap.observed_landmarks_ids.erase( unique( lmap.observed_landmarks_ids.begin(), lmap.observed_landmarks_ids.end() ), 
                                                        lmap.observed_landmarks_ids.end() );

    }

    void condensed_optimizer::local_maps_manager(){

        if(maps_.empty()){ 
            // if the maps vector is empty create the first local map object.
            // the first local map will have the first frame ptr equal to the origin ptr
            
            local_map lmap(frames_vector_ptr_->back(),maps_.size());
            lmap.origin_ptr = frames_vector_ptr_->back();
            maps_.push_back(lmap);
            return;
        }

        else if(frames_vector_ptr_->back()->seq % 9 == 0 && maps_.back().last_frame_ptr==nullptr){
            // Every 10 frames write the 10th frame as the last of the current local map and the first of the new local map
            
            maps_.back().last_frame_ptr = frames_vector_ptr_->back();
            write_observed_landmarks(maps_.back());
            write_landmarks_separators();
            last_map_verbose();
            optimize_local_map();
            local_map lmap(frames_vector_ptr_->back(),maps_.size());            
            maps_.push_back(lmap);

            return;
        }

        
        else if(frames_vector_ptr_->back()->seq - maps_.back().first_frame_ptr->seq == 4 && maps_.back().origin_ptr==nullptr){
            // if the new local map is not the first one to be written, save the origin when you reveive the 5th frame 
            maps_.back().origin_ptr = frames_vector_ptr_->back();
            return;
        }
    }


    void condensed_optimizer::write_landmarks_separators(){
            
        if (maps_.size() < 2){
            return;
        }

        int index = maps_.size() - 2; // second last local map in the vector
        local_map* map_to_be_written; 
        local_map* previous_one;
        local_map* next_one;;

        map_to_be_written = &maps_[index];
        previous_one = &maps_[index-1];
        next_one = &maps_[index+1];

        std::vector<long unsigned> previous_intersection;
        std::vector<long unsigned> final_intersection;

        if (index == 0){
            previous_intersection = map_to_be_written->observed_landmarks_ids;
        } else{
        std::set_intersection(map_to_be_written->observed_landmarks_ids.begin(), map_to_be_written->observed_landmarks_ids.end(), 
                                previous_one->observed_landmarks_ids.begin(), previous_one->observed_landmarks_ids.end(), 
                                std::back_inserter(previous_intersection));
        }

        std::set_intersection(next_one->observed_landmarks_ids.begin(), next_one->observed_landmarks_ids.end(), 
                                previous_intersection.begin(), previous_intersection.end(), 
                                std::back_inserter(final_intersection));

        for ( int i = 0; i < final_intersection.size(); i ++){

            map_to_be_written->separators_mappoints.push_back(landmarks_map_->map_points.at(final_intersection[i]));

        }

    }

    void condensed_optimizer::optimize_local_map(){

        typedef g2o::BlockSolver< g2o::BlockSolverTraits<3,3> > Block;
        Block::LinearSolverType* linearSolver = new g2o::LinearSolverEigen<Block::PoseMatrixType>();
        Block* solver_ptr = new Block( std::unique_ptr<Block::LinearSolverType>(linearSolver) );
        g2o::OptimizationAlgorithmLevenberg* algorithm = new g2o::OptimizationAlgorithmLevenberg(std::unique_ptr<Block>(solver_ptr) );
        g2o::SparseOptimizer optimizer;
        optimizer.setAlgorithm( algorithm );

        std::map<int,int> camera_to_graph;
        std::map<unsigned long,int> landmark_to_graph;
        int j = 0;
        
        for (int i = maps_.back().first_frame_ptr->seq; i < maps_.back().last_frame_ptr->seq + 1; i++){
            
            VertexRobotSE2* robot = new VertexRobotSE2();
            robot->setId(j);
            if(maps_.back().origin_ptr->seq == i){
                robot->setFixed(1);
            }
            robot->setEstimate((*frames_vector_ptr_)[i]->robot_pose);
            robot->p_matrix = (*frames_vector_ptr_)[i]->p_matrix;
            optimizer.addVertex(robot);   
            camera_to_graph.insert(std::make_pair(i,j));          
            j++;
        }
        for ( int i = 0; i < maps_.back().observed_landmarks_ids.size(); i++){

            g2o::VertexPointXYZ* landmark = new g2o::VertexPointXYZ();
            landmark->setId(j);
            landmark->setMarginalized(true);
            landmark->setEstimate(landmarks_map_->map_points.at(maps_.back().observed_landmarks_ids[i])->p_world);
            optimizer.addVertex(landmark);
            landmark_to_graph.insert(std::make_pair(maps_.back().observed_landmarks_ids[i],j));
            j++;
        }
        
        for (int i = maps_.back().first_frame_ptr->seq; i < maps_.back().last_frame_ptr->seq + 1; i++){

            for (auto& observations: (*frames_vector_ptr_)[i]->observed_points){

                EdgeSE2projectXYZ* landmark_projection_edge = new EdgeSE2projectXYZ();
                landmark_projection_edge -> setVertex( 0, dynamic_cast<g2o::VertexPointXYZ*>   (optimizer.vertex(landmark_to_graph.at(observations.second->id))) );
                landmark_projection_edge -> setVertex( 1, dynamic_cast<VertexRobotSE2*>   (optimizer.vertex(camera_to_graph.at(i)) ) );
                landmark_projection_edge -> setMeasurement (Eigen::Vector2d((*frames_vector_ptr_)[i]->kpts[observations.first].x,(*frames_vector_ptr_)[i]->kpts[observations.first].y));
                landmark_projection_edge -> setRobustKernel( new g2o::RobustKernelHuber() );
                landmark_projection_edge -> setInformation( Eigen::Matrix2d::Identity() );
                optimizer.addEdge( landmark_projection_edge );
            }

            if ( i != maps_.back().first_frame_ptr->seq ){

                EdgeSE2Custom* odometry_edge = new EdgeSE2Custom();
                odometry_edge -> setVertex( 0, dynamic_cast<VertexRobotSE2*>   (optimizer.vertex(camera_to_graph.at(i-1))));
                odometry_edge -> setVertex( 1, dynamic_cast<VertexRobotSE2*>   (optimizer.vertex(camera_to_graph.at(i))));
                odometry_edge -> setMeasurement ((*frames_vector_ptr_)[i]->odom_displ);
                odometry_edge -> setRobustKernel( new g2o::RobustKernelHuber() );
                odometry_edge -> setInformation( Eigen::Matrix3d::Identity() );
                optimizer.addEdge( odometry_edge );
            }

        }
        // optimizer.setVerbose(true);
        optimizer.initializeOptimization();
        optimizer.optimize(5);
        // update current estimates with optimization results
        for (int i = maps_.back().first_frame_ptr->seq; i < maps_.back().last_frame_ptr->seq + 1; i++){
            VertexRobotSE2* pose_update = static_cast< VertexRobotSE2*> ( optimizer.vertex(camera_to_graph.at(i)) );  
            (*frames_vector_ptr_)[i]->robot_pose = pose_update->estimate();
        }
        for ( const auto &myPair : landmark_to_graph ){
            g2o::VertexPointXYZ* landmark_update = static_cast< g2o::VertexPointXYZ*> ( optimizer.vertex( landmark_to_graph.at(myPair.first) ) ); 
            landmarks_map_->map_points.at(myPair.first)->p_world = landmark_update->estimate();
        }
    }


}