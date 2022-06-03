#include <condensed_optimizer.hpp>

namespace planar_monocular_slam{

    void condensed_optimizer::write_observed_landmarks(local_map& lmap){

        for (int i = lmap.first_frame_ptr->seq; i < (lmap.last_frame_ptr->seq + 1); i++){
            
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
            lmap.origin_ptr = lmap.first_frame_ptr;
            maps_.push_back(lmap);
            return;
        }

        else if(frames_vector_ptr_->back()->seq % 9 == 0 && maps_.back().last_frame_ptr==nullptr){
            // Every 10 frames write the 10th frame as the last of the current local map and the first of the new local map
            
            maps_.back().last_frame_ptr = frames_vector_ptr_->back();                        
            last_map_verbose();
            local_map lmap(frames_vector_ptr_->back(),maps_.size());            
            maps_.push_back(lmap);

            return;
        }

        else if (frames_vector_ptr_->back()->seq - maps_.back().first_frame_ptr->seq == 1 && maps_.back().seq != 0){

            write_observed_landmarks(maps_[maps_.size()-2]);
            // ROS_INFO_STREAM("Writing observed landmarks of map " << maps_.size()-2);
            write_landmarks_separators();
            
            if (maps_.size() > 2){
                // ROS_INFO_STREAM("Solving map optimization of " << maps_.size()-3);
                optimize_local_map(maps_[maps_.size()-3]);
            }
            
            if (maps_.size() > 3){
                global_optimization();
            }
            return;
        }

        
        else if(frames_vector_ptr_->back()->seq - maps_.back().first_frame_ptr->seq == 4 && maps_.back().origin_ptr==nullptr){
            // if the new local map is not the first one to be written, save the origin when you reveive the 5th frame 
            maps_.back().origin_ptr = frames_vector_ptr_->back();
            return;
        }
    }


    void condensed_optimizer::write_landmarks_separators(){
            
        if (maps_.size() < 3){
            return;
        }
        // ROS_INFO_STREAM("Writing landmarks separators of map " << maps_.size()-3);
        int index = maps_.size() - 3; // second last local map in the vector
        local_map* map_to_be_written; 
        local_map* previous_one;
        local_map* next_one;

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

        map_to_be_written->separators_mappoints = final_intersection;

    }

    void condensed_optimizer::pose_unscented_mapping (const Eigen::Matrix3d cov, const Eigen::Vector3d new_mean, Eigen::Matrix3d new_cov,
	                                        const Eigen::Isometry2d gauge, const Eigen::Isometry2d separator) {
    typedef g2o::SigmaPoint<Eigen::Vector3d> MySigmaPoint;

    std::vector<MySigmaPoint, Eigen::aligned_allocator<MySigmaPoint> > spts;
    Eigen::Vector3d mean;
    mean.setZero();

    sampleUnscented(spts, mean, cov);

    // now apply the oplus operator to the sigma points,
    // and get the points in the global space

    std::vector<MySigmaPoint, Eigen::aligned_allocator<MySigmaPoint> >
        tspts = spts;

    for (int j = 0; j < spts.size(); j++) {

    tspts[j]._sample = t2v( v2t(new_mean).inverse() * gauge.inverse() * v2t (spts[j]._sample) * separator ) ;

    }

    reconstructGaussian(mean, new_cov, tspts);
    }

    void condensed_optimizer::landmark_unscented_mapping (const Eigen::Matrix3d cov, Eigen::Matrix3d new_cov,
	                                        const Eigen::Isometry2d gauge){
    typedef g2o::SigmaPoint<Eigen::Vector3d> MySigmaPoint;
    std::vector<MySigmaPoint, Eigen::aligned_allocator<MySigmaPoint> > spts;
    Eigen::Vector3d mean;
    mean.setZero();

    sampleUnscented(spts, mean, cov);

    // now apply the oplus operator to the sigma points,
    // and get the points in the global space

    std::vector<MySigmaPoint, Eigen::aligned_allocator<MySigmaPoint> >
        tspts = spts;

    for (int j = 0; j < spts.size(); j++) {

    tspts[j]._sample = t2t3d(gauge).inverse() * spts[j]._sample ;

    }

    reconstructGaussian(mean, new_cov, tspts);
    }

    bool condensed_optimizer::covariance_condition_check( 	const g2o::SparseBlockMatrix< Eigen::MatrixXd >& marginals,
						                const g2o::SparseOptimizer& optimizer,
						                int graph_index)
    {

        Eigen::Matrix3d covariance = marginals.block(  optimizer.vertex(graph_index)->hessianIndex(),
                                                    optimizer.vertex(graph_index)->hessianIndex())->eval();

        double condition_number = covariance.eigenvalues().real().maxCoeff()/covariance.eigenvalues().real().minCoeff();
        if(condition_number > 10e5 ){
            return false;}
        else{
            return true;
        }

    }

    void condensed_optimizer::optimize_local_map( local_map& lmap){

        typedef g2o::BlockSolver< g2o::BlockSolverTraits<3,3> > Block;
        // typedef g2o::BlockSolver<g2o::BlockSolverTraits<3, 3> > BlockSolver_3_3;
        // Block::LinearSolverType* linearSolver = new g2o::LinearSolverEigen<Block::PoseMatrixType>();
        // Block::LinearSolverType* linearSolver = new g2o::LinearSolverCSparse<Block::PoseMatrixType>();
        // Block::LinearSolverType* linearSolver = new g2o::LinearSolverPCG<Block::PoseMatrixType>();
        Block::LinearSolverType* linearSolver = new g2o::LinearSolverCholmod<Block::PoseMatrixType>();
        Block* solver_ptr = new Block( std::unique_ptr<Block::LinearSolverType>(linearSolver) );
        
        // g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(
        //                 std::unique_ptr<BlockSolver_3_3>(new BlockSolver_3_3(std::unique_ptr<BlockSolver_3_3::LinearSolverType>(
        //                 new g2o::LinearSolverCholmod<BlockSolver_3_3::PoseMatrixType>()))));

        
        g2o::OptimizationAlgorithmLevenberg* algorithm = new g2o::OptimizationAlgorithmLevenberg(std::unique_ptr<Block>(solver_ptr) );
        

        g2o::SparseOptimizer optimizer;
        optimizer.setAlgorithm( algorithm );

        std::map<int,int> camera_to_graph;
        std::map<unsigned long,int> landmark_to_graph;
        int j = 0;
        // ROS_INFO_STREAM(lmap.first_frame_ptr->seq);
        // ROS_INFO_STREAM(lmap.last_frame_ptr->seq);
        for (int i = lmap.first_frame_ptr->seq; i < (lmap.last_frame_ptr->seq + 1); i++){
            
            VertexRobotSE2* robot = new VertexRobotSE2();
            robot->setId(j);
            robot->setFixed( i == lmap.origin_ptr->seq);
            robot->setEstimate((*frames_vector_ptr_)[i]->robot_pose);
            robot->p_matrix = (*frames_vector_ptr_)[i]->p_matrix;
            optimizer.addVertex(robot);   
            camera_to_graph.insert(std::make_pair(i,j));
            j++;
        }
        // ROS_INFO_STREAM(lmap.observed_landmarks_ids.size());
        for ( int i = 0; i < lmap.observed_landmarks_ids.size(); i++){
            // ROS_INFO_STREAM(lmap.observed_landmarks_ids[i]);
            g2o::VertexPointXYZ* landmark = new g2o::VertexPointXYZ();
            landmark->setId(j);
            landmark->setFixed(false);
            if (std::find(lmap.separators_mappoints.begin(),lmap.separators_mappoints.end(),lmap.observed_landmarks_ids[i]) == lmap.separators_mappoints.end()){
                landmark->setMarginalized(true);}
            landmark->setEstimate(landmarks_map_->map_points.at(lmap.observed_landmarks_ids[i])->p_world);
            optimizer.addVertex(landmark);
            landmark_to_graph.insert(std::make_pair(lmap.observed_landmarks_ids[i],j));
            j++;
        }
        
        for (int i = lmap.first_frame_ptr->seq; i < (lmap.last_frame_ptr->seq + 1); i++){

            for (auto& observations: frames_vector_ptr_->at(i)->observed_points){
                // ROS_INFO("----------------------");
                // ROS_INFO_STREAM(i);
                // ROS_INFO_STREAM(observations.second->id);
                // ROS_INFO_STREAM(landmark_to_graph.at(observations.second->id));
                EdgeSE2projectXYZ* landmark_projection_edge = new EdgeSE2projectXYZ();
                landmark_projection_edge -> setVertex( 0, dynamic_cast<g2o::VertexPointXYZ*>   (optimizer.vertex(landmark_to_graph.at(observations.second->id))) );
                landmark_projection_edge -> setVertex( 1, dynamic_cast<VertexRobotSE2*>   (optimizer.vertex(camera_to_graph.at(i)) ) );
                landmark_projection_edge -> setMeasurement (Eigen::Vector2d((*frames_vector_ptr_)[i]->kpts[observations.first].x,(*frames_vector_ptr_)[i]->kpts[observations.first].y));
                landmark_projection_edge -> setRobustKernel( new g2o::RobustKernelHuber() );
                landmark_projection_edge -> setInformation( Eigen::Matrix2d::Identity() );
                optimizer.addEdge( landmark_projection_edge );
            }

            if ( i != lmap.first_frame_ptr->seq ){

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
        optimizer.optimize(10);

        // update current estimates with optimization results
        for (int i = lmap.first_frame_ptr->seq; i < (lmap.last_frame_ptr->seq + 1); i++){
            VertexRobotSE2* pose_update = static_cast< VertexRobotSE2*> ( optimizer.vertex(camera_to_graph.at(i)) );  
            frames_vector_ptr_->at(i)->robot_pose = pose_update->estimate();
        }
        for ( const auto &myPair : landmark_to_graph ){
            g2o::VertexPointXYZ* landmark_update = static_cast< g2o::VertexPointXYZ*> ( optimizer.vertex( landmark_to_graph.at(myPair.first) ) ); 
            landmarks_map_->map_points.at(myPair.first)->p_world = landmark_update->estimate();
        }

        // compute marginals
        /////////////////////////////////////////////////////////////////////////////////////////////////////
        g2o::SparseBlockMatrix< Eigen::MatrixXd > marginals;
        g2o::OptimizableGraph::VertexContainer v_k; 
        
        if(lmap.seq != 0){
            v_k.push_back(optimizer.vertex(camera_to_graph.at(lmap.first_frame_ptr->seq)));
            v_k.push_back(optimizer.vertex(camera_to_graph.at(lmap.last_frame_ptr->seq)));
        } else {
            v_k.push_back(optimizer.vertex(camera_to_graph.at(lmap.last_frame_ptr->seq)));
        }
        for (int i = 0; i < lmap.separators_mappoints.size(); i ++){

            v_k.push_back(optimizer.vertex(landmark_to_graph.at(lmap.separators_mappoints.at(i))));

        }

        // if (optimizer.verifyInformationMatrices(true)){
        //     ROS_INFO("Information matrices are PSD");}
        // if (optimizer.gaugeFreedom()){
        //     ROS_INFO(" The optimizer has gauge freedom");}

        // condensed_pairs.push_back(std::pair<int,int>(30,30));
        // ROS_INFO("Computing marginals...");
        optimizer.computeMarginals(marginals ,v_k);
        // ROS_INFO("Marginals computed!");
        // ROS_INFO_STREAM(marginals);
        
        // std::cin.get();
        // add vertices and edges to the global optimizer
        ////////////////////////////////////////////////////////////////////////////////////////////////////

        // add origin as an SE2 vertex
        g2o::VertexSE2* origin = new g2o::VertexSE2();
        origin->setId(global_opt_.latest_vertex_index);
        origin->setFixed(global_opt_.latest_vertex_index == 0);        
        origin->setEstimate( g2o::SE2(lmap.origin_ptr->robot_pose));
        global_opt_.optimizer.addVertex(origin);
        global_opt_.newVertices.insert(origin);
        global_opt_.camera_to_graph.insert(std::make_pair(lmap.origin_ptr->seq,global_opt_.latest_vertex_index));
        global_opt_.latest_vertex_index++;

        // add first frame (only if it hasn't already been added)
        if( lmap.seq != 0 && !global_opt_.camera_to_graph.count(lmap.first_frame_ptr->seq)){

            g2o::VertexSE2* first = new g2o::VertexSE2();
            first->setId(global_opt_.latest_vertex_index);
            // first->setFixed(global_opt_.latest_vertex_index);        
            first->setEstimate( g2o::SE2(lmap.first_frame_ptr->robot_pose));
            global_opt_.optimizer.addVertex(first);
            global_opt_.newVertices.insert(first);
            global_opt_.camera_to_graph.insert(std::make_pair(lmap.first_frame_ptr->seq,global_opt_.latest_vertex_index));
            global_opt_.latest_vertex_index++;

        }

        // add last frame (if it hasn't already been added)
        if( !global_opt_.camera_to_graph.count(lmap.last_frame_ptr->seq)){

            g2o::VertexSE2* last = new g2o::VertexSE2();
            last->setId(global_opt_.latest_vertex_index);
            // last->setFixed(global_opt_.latest_vertex_index );        
            last->setEstimate( g2o::SE2(lmap.last_frame_ptr->robot_pose));
            global_opt_.optimizer.addVertex(last);
            global_opt_.newVertices.insert(last);
            global_opt_.camera_to_graph.insert(std::make_pair(lmap.last_frame_ptr->seq,global_opt_.latest_vertex_index));
            global_opt_.latest_vertex_index++;

        }

        // add map points vertices (if they haven't already been added)
        for( int i = 0; i < lmap.separators_mappoints.size(); i ++){

            if ( !covariance_condition_check(marginals,optimizer,landmark_to_graph.at(lmap.separators_mappoints[i]))) {

                global_opt_.outliers_landmarks.push_back(lmap.separators_mappoints[i]);
                continue;
            }

            if(!global_opt_.landmark_to_graph.count(lmap.separators_mappoints[i]) && 
                std::find(global_opt_.outliers_landmarks.begin(), global_opt_.outliers_landmarks.end(), lmap.separators_mappoints[i]) 
                        == global_opt_.outliers_landmarks.end()){

                g2o::VertexPointXYZ* landmark = new g2o::VertexPointXYZ();
                landmark->setId(global_opt_.latest_vertex_index);
                // landmark->setMarginalized(true);
                landmark->setEstimate(landmarks_map_->map_points.at(lmap.separators_mappoints[i])->p_world);
                global_opt_.optimizer.addVertex(landmark);
                global_opt_.newVertices.insert(landmark);
                global_opt_.landmark_to_graph.insert(std::make_pair(lmap.separators_mappoints[i],global_opt_.latest_vertex_index));
                global_opt_.latest_vertex_index++;
            }
        }

        // add condensed measurements: first frame - origin frame
        Eigen::Matrix3d covariance;
        Eigen::Matrix3d new_cov;

        if (lmap.seq != 0){

            g2o::EdgeSE2* first_to_origin = new g2o::EdgeSE2();
            // ROS_INFO_STREAM("Adding condensed measurements of pose " << lmap.first_frame_ptr->seq);
            Eigen::Isometry2d mean = lmap.origin_ptr->robot_pose.inverse()*lmap.first_frame_ptr->robot_pose;
            
            covariance = marginals.block(      optimizer.vertex(camera_to_graph.at(lmap.first_frame_ptr->seq))->hessianIndex(),
                                                optimizer.vertex(camera_to_graph.at(lmap.first_frame_ptr->seq))->hessianIndex())->eval();
            pose_unscented_mapping (covariance, t2v(mean), new_cov,
	                                        lmap.origin_ptr->robot_pose, lmap.first_frame_ptr->robot_pose);
                                        
            first_to_origin->setVertex(0, dynamic_cast<g2o::VertexSE2*> (global_opt_.optimizer.vertex(global_opt_.camera_to_graph.at(lmap.origin_ptr->seq))));
            first_to_origin->setVertex(1, dynamic_cast<g2o::VertexSE2*> (global_opt_.optimizer.vertex(global_opt_.camera_to_graph.at(lmap.first_frame_ptr->seq))));
            first_to_origin->setMeasurement(g2o::SE2(mean));
            first_to_origin->setRobustKernel( new g2o::RobustKernelHuber() );
            first_to_origin->setInformation(new_cov.inverse().eval());
            // first_to_origin->setInformation(Eigen::Matrix3d::Identity());

            global_opt_.optimizer.addEdge(first_to_origin);
            global_opt_.newEdges.insert(first_to_origin);

        }

        // add condensed measurements: last frame - origin frame
        g2o::EdgeSE2* last_to_origin = new g2o::EdgeSE2();
        // ROS_INFO_STREAM("Adding condensed measurements of pose " << lmap.last_frame_ptr->seq);
        Eigen::Isometry2d mean = lmap.origin_ptr->robot_pose.inverse()*lmap.last_frame_ptr->robot_pose;
        covariance = marginals.block(  optimizer.vertex(camera_to_graph.at(lmap.last_frame_ptr->seq))->hessianIndex(),
                                        optimizer.vertex(camera_to_graph.at(lmap.last_frame_ptr->seq))->hessianIndex())->eval();                                                                
        pose_unscented_mapping (covariance, t2v(mean), new_cov,
	                                        lmap.origin_ptr->robot_pose, lmap.last_frame_ptr->robot_pose);
        last_to_origin->setVertex(0, dynamic_cast<g2o::VertexSE2*> (global_opt_.optimizer.vertex(global_opt_.camera_to_graph.at(lmap.origin_ptr->seq))));
        last_to_origin->setVertex(1, dynamic_cast<g2o::VertexSE2*> (global_opt_.optimizer.vertex(global_opt_.camera_to_graph.at(lmap.last_frame_ptr->seq))));
        last_to_origin->setMeasurement(g2o::SE2(mean));
        last_to_origin->setRobustKernel( new g2o::RobustKernelHuber() );
        last_to_origin->setInformation(new_cov.inverse().eval());

        global_opt_.optimizer.addEdge(last_to_origin);
        global_opt_.newEdges.insert(last_to_origin);

        // add condensed measurements: landmark - origin frame
        for (int i = 0; i < lmap.separators_mappoints.size(); i++){

            if (std::find(global_opt_.outliers_landmarks.begin(), global_opt_.outliers_landmarks.end(), lmap.separators_mappoints[i]) 
                        != global_opt_.outliers_landmarks.end()){
                            continue;
                        }

            // ROS_INFO_STREAM("Adding condensed measurements of landmark " << lmap.separators_mappoints[i]);
            covariance = marginals.block(       optimizer.vertex(landmark_to_graph.at(lmap.separators_mappoints[i]))->hessianIndex(),
                                                optimizer.vertex(landmark_to_graph.at(lmap.separators_mappoints[i]))->hessianIndex())->eval();
            
            EdgeSE2landmarkXYZ* landmark_condensed_edge = new EdgeSE2landmarkXYZ();
            Eigen::Vector3d mean_lm = t2t3d(lmap.origin_ptr->robot_pose).inverse() * landmarks_map_->map_points.at(lmap.separators_mappoints[i])->p_world;            
            landmark_unscented_mapping (covariance, new_cov,
	                                         lmap.origin_ptr->robot_pose);
            landmark_condensed_edge->setVertex(0, dynamic_cast<g2o::VertexPointXYZ*> (global_opt_.optimizer.vertex(global_opt_.landmark_to_graph.at(lmap.separators_mappoints[i]))));
            landmark_condensed_edge->setVertex(1, dynamic_cast<g2o::VertexSE2*> (global_opt_.optimizer.vertex(global_opt_.camera_to_graph.at(lmap.origin_ptr->seq))));
            landmark_condensed_edge->setMeasurement(mean_lm);
            landmark_condensed_edge->setRobustKernel( new g2o::RobustKernelHuber() );
            landmark_condensed_edge->setInformation(new_cov.inverse().eval());
            global_opt_.optimizer.addEdge(landmark_condensed_edge);
            global_opt_.newEdges.insert(landmark_condensed_edge);

        }

    }

    void condensed_optimizer::global_optimization(){

        if(global_opt_.initialized){
            global_opt_.optimizer.updateInitialization(global_opt_.newVertices,global_opt_.newEdges);
            global_opt_.newEdges.clear();
            global_opt_.newVertices.clear();
            global_opt_.optimizer.optimize(50,true);
        
        }else{
            global_opt_.optimizer.initializeOptimization();
            global_opt_.initialized = true;
            global_opt_.newEdges.clear();
            global_opt_.newVertices.clear();

            global_opt_.optimizer.optimize(20);
        }
        

        for (const auto &myPair : global_opt_.camera_to_graph){

            g2o::VertexSE2* updated_pose = static_cast< g2o::VertexSE2*> ( global_opt_.optimizer.vertex(myPair.second) ); 
            frames_vector_ptr_->at(myPair.first)->robot_pose = updated_pose->estimate().toIsometry();
        }

        for (const auto &myPair : global_opt_.landmark_to_graph){
            
            g2o::VertexPointXYZ* updated_landmark = static_cast< g2o::VertexPointXYZ*> ( global_opt_.optimizer.vertex(myPair.second) ); 
            landmarks_map_->map_points.at(myPair.first)->p_world = updated_landmark->estimate();

        }

    }

}