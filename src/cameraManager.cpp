#include <cameraManager.hpp>

namespace planar_monocular_slam{

    void CameraManager::addCamera(cv::Mat& image,Eigen::Isometry2d& pose){
        cv::Mat clahe_image;
        std::vector<cv::KeyPoint> kpts;
        cv::Mat dscs;
        clahe_->apply(image,clahe_image);
        orb_->detect(clahe_image, kpts);
        orb_->compute(clahe_image, kpts, dscs);

        std::vector<cv::Point2f> kpts_converted;

        for(int i = 0; i < kpts.size(); i++){
            kpts_converted.push_back(kpts[i].pt);
        }
        
        Eigen::Matrix<double, 3, 4> proj_mat = P_*extrinsics_.inverse()*(t2t3d(pose)).inverse().matrix();

        Camera::Ptr cam( new Camera(dscs,kpts_converted,pose,proj_mat));
        
        for (int i = 0; i < cam->kpts.size();i++){
            cam->unassigned_kpts.push_back(i);
        }
        cam->seq = this->n_of_cams();
        if ( (this->n_of_cams()) == 0){
            cam->odom_displ.setIdentity();
        } else{
            cam->odom_displ = t2v(camera_vector_.back()->robot_pose.inverse()*pose);
        }
        camera_vector_.push_back(cam);

        if ( (this->n_of_cams()) > 1 ) {
        matchNewKeypoints(); // initialize new map points;
                             // add new observations to previously found map points;
        }
    }

    bool CameraManager::checkTransform(const cv::Mat& image){

        cv::Mat clahe_image;
        clahe_->apply(image,clahe_image);

        // Find matches with last frame
        std::vector<cv::KeyPoint> kpts_current;
        std::vector<cv::Point2f> pts_last,pts_current;
        cv::Mat dscs_current;

        std::vector< std::vector< cv::DMatch > > matches_feats;
        std::vector<cv::DMatch> matches;

        cv::Mat F,H;
        cv::Mat f_mask,h_mask;

        orb_->detect(clahe_image, kpts_current);
        orb_->compute(clahe_image, kpts_current, dscs_current);

        matcher_.knnMatch(dscs_current,camera_vector_.back()->dscs,matches_feats,2,cv::noArray());
        filterMatches(matches_feats, matches);

        if (matches.size() == 0){
            return true;
        }

        for (int i = 0; i < matches.size(); i ++){
            pts_current.push_back(kpts_current[matches[i].queryIdx].pt);
            pts_last.push_back(camera_vector_.back()->kpts[matches[i].trainIdx]);
        }

        //compute Fundamental with previous frame

        F = cv::findFundamentalMat(pts_last,pts_current,f_mask,cv::FM_RANSAC,1.,0.99);
        int f_inliers = cv::sum(f_mask)[0];

        //compute homography with previous frame

        H = cv::findHomography(pts_last,pts_current,h_mask,cv::RANSAC,3.);
        int h_inliers = cv::sum(h_mask)[0];

        //compare number of inliers and return results
        if (f_inliers > h_inliers){
            latest_inliers_ = f_mask;
            return true;
        } else {
            return false;
        }        

    }

    bool CameraManager::checkTransform(const geometry_msgs::Pose& odom_pose, const cv::Mat& image){

        Eigen::Isometry3d pose_eigen;
        Eigen::Isometry2d pose_eigen_2d;
        Eigen::Vector3d odom_displ;
        tf::poseMsgToEigen(odom_pose,pose_eigen);
        pose_eigen_2d = t3t2d(pose_eigen);

        odom_displ = t2v(camera_vector_.back()->robot_pose.inverse()*pose_eigen_2d);

        if( odom_displ.head(2).norm() > 0.1){
            cv::Mat clahe_image;
            clahe_->apply(image,clahe_image);

            // Find matches with last frame
            std::vector<cv::KeyPoint> kpts_current;
            std::vector<cv::Point2f> pts_last,pts_current;
            cv::Mat dscs_current;

            std::vector< std::vector< cv::DMatch > > matches_feats;
            std::vector<cv::DMatch> matches;

            cv::Mat F;
            cv::Mat f_mask;

            orb_->detect(clahe_image, kpts_current);
            orb_->compute(clahe_image, kpts_current, dscs_current);

            matcher_.knnMatch(dscs_current,camera_vector_.back()->dscs,matches_feats,2,cv::noArray());
            filterMatches(matches_feats, matches);

            if (matches.size() == 0){
                return true;
            }

            for (int i = 0; i < matches.size(); i ++){
                pts_current.push_back(kpts_current[matches[i].queryIdx].pt);
                pts_last.push_back(camera_vector_.back()->kpts[matches[i].trainIdx]);
            }

            //compute Fundamental with previous frame

            F = cv::findFundamentalMat(pts_last,pts_current,f_mask,cv::FM_RANSAC,1.,0.99);
            latest_inliers_ = f_mask;
            return true;
        }else {
            return false;
        }

    }

    void CameraManager::filterMatches(const std::vector<std::vector<cv::DMatch> >& matches_feats,std::vector<cv::DMatch>& matches){    
        // Clearing the current matches vector
        matches.clear();
        // Filtering matches according to the ratio test
        for (unsigned m = 0; m < matches_feats.size(); m++) {
            if (matches_feats[m][0].distance <= matches_feats[m][1].distance * 0.8) {
            matches.push_back(matches_feats[m][0]);
                }
        }
    }
    
    bool CameraManager::isInFrame ( const Eigen::Vector3d& pt_world ){

        Eigen::Vector3d p_pixel = camera_vector_.back()->p_matrix*Eigen::Vector4d(pt_world.x(),pt_world.y(),pt_world.z(),1.);
        if ( p_pixel.z()<0 ) return false;
        p_pixel /= p_pixel.z();
        return p_pixel.x()>0 && p_pixel.y()>0 
            && p_pixel.y()<height_ 
            && p_pixel.x()<width_;

    }


    void  CameraManager::matchNewKeypoints(){

        std::vector<std::vector<cv::DMatch>> matches_feats;
        std::vector<cv::DMatch> matches;

        Camera::Ptr current_frame = camera_vector_.back();
        Camera::Ptr last_frame = camera_vector_[camera_vector_.size()-2]; 

        if (!map_->map_points.empty()){

            // select the candidates in map 
            cv::Mat candidate_dscs;
            std::vector<MapPoint::Ptr> candidate_points;
            for ( auto& allpoints: map_->map_points )
            {
                MapPoint::Ptr& p = allpoints.second;
                // check if p is in curr frame image 
                if ( this->isInFrame(p->p_world) )
                {
                    // add to candidate 
                    candidate_points.push_back( p );
                    candidate_dscs.push_back( p->descriptor );
                }
            }
            if(candidate_points.size()>0){
                matcher_.knnMatch(candidate_dscs,current_frame->dscs,matches_feats,2,cv::noArray());
                // std::cout << "matching with already triangulated points" << std::endl;
                filterMatches(matches_feats, matches);
                for(int i = 0; i < matches.size(); i++){

                    candidate_points[matches[i].queryIdx]->insert_frame(current_frame.get());
                    current_frame->unassigned_kpts.erase(std::remove(current_frame->unassigned_kpts.begin(), 
                                                                current_frame->unassigned_kpts.end(), matches[i].trainIdx), current_frame->unassigned_kpts.end());                
                    current_frame->observed_points.insert(std::make_pair(matches[i].trainIdx,candidate_points[matches[i].queryIdx]));
                }
            }
        }
        
        matches_feats.clear();
        
        matcher_.knnMatch(current_frame->dscs,last_frame->dscs,matches_feats,2,cv::noArray());
        // std::cout << "matching with previous frame" << std::endl;
        filterMatches(matches_feats, matches);
        for (int i = 0; i < matches.size(); i ++){
            if (last_frame->observed_points.find(matches[i].trainIdx) == last_frame->observed_points.end() &&
                std::find(current_frame->unassigned_kpts.begin(),current_frame->unassigned_kpts.end(),(matches[i].queryIdx)) != current_frame -> unassigned_kpts.end()  &&
                latest_inliers_.at<uchar>(i)==1          
            ){
                    Eigen::Vector3d p = LinearLSTriangulation(current_frame->kpts[matches[i].queryIdx],current_frame->p_matrix,
                                                                last_frame->kpts[matches[i].trainIdx],last_frame->p_matrix);
                    if ( !this->isInFrame(p) ){
                        continue;
                    }
                    MapPoint::Ptr map_point (new MapPoint(
                                    last_triangulated_point, p, current_frame->dscs.row(matches[i].queryIdx).clone(), current_frame.get()));
                    map_point->insert_frame(last_frame.get());
                    current_frame->observed_points.insert(std::make_pair(matches[i].queryIdx,map_point));
                    last_frame->observed_points.insert(std::make_pair(matches[i].trainIdx,map_point));
                    map_->insertMapPoint(map_point);
                    
                    current_frame->unassigned_kpts.erase(std::remove(current_frame->unassigned_kpts.begin(), 
                                                            current_frame->unassigned_kpts.end(), matches[i].queryIdx), current_frame->unassigned_kpts.end());
                    last_frame->unassigned_kpts.erase(std::remove(last_frame->unassigned_kpts.begin(), 
                                                            last_frame->unassigned_kpts.end(), matches[i].trainIdx), last_frame->unassigned_kpts.end());
                    last_triangulated_point++;
                }
        }
    }

    /**
     From "Triangulation", Hartley, R.I. and Sturm, P., Computer vision and image understanding, 1997
    */
    Eigen::Vector3d CameraManager::LinearLSTriangulation(   const cv::Point2f& img_pt1,       // image point (u,v)
                                                            const Eigen3_4d& proj_mat1,       //camera 1 matrix
                                                            const cv::Point2f& img_pt2,      //image point in 2nd camera
                                                            const Eigen3_4d& proj_mat2       //camera 2 matrix
                                                        )
    {
        Eigen::Vector3d pt1(img_pt1.x,img_pt1.y,1);
        Eigen::Vector3d pt2(img_pt2.x,img_pt2.y,1);
        Eigen4_3d A; 
        Eigen::Vector4d b;
        Eigen::Vector3d x;
        //build matrix A for homogenous equation system Ax = 0
        //assume X = (x,y,z,1), for Linear-LS method
        //which turns it into a AX = B system, where A is 4x3, X is 3x1 and B is 4x1
        A <<  pt1.x()*proj_mat1(2,0)-proj_mat1(0,0),    pt1.x()*proj_mat1(2,1)-proj_mat1(0,1),      pt1.x()*proj_mat1(2,2)-proj_mat1(0,2),
            pt1.y()*proj_mat1(2,0)-proj_mat1(1,0),    pt1.y()*proj_mat1(2,1)-proj_mat1(1,1),      pt1.y()*proj_mat1(2,2)-proj_mat1(1,2),
            pt2.x()*proj_mat2(2,0)-proj_mat2(0,0), pt2.x()*proj_mat2(2,1)-proj_mat2(0,1),   pt2.x()*proj_mat2(2,2)-proj_mat2(0,2),
            pt2.y()*proj_mat2(2,0)-proj_mat2(1,0), pt2.y()*proj_mat2(2,1)-proj_mat2(1,1),   pt2.y()*proj_mat2(2,2)-proj_mat2(1,2);

        b <<    -(pt1.x()*proj_mat1(2,3)    -proj_mat1(0,3)),
                -(pt1.y()*proj_mat1(2,3)  -proj_mat1(1,3)),
                -(pt2.x()*proj_mat2(2,3)    -proj_mat2(0,3)),
                -(pt2.y()*proj_mat2(2,3)    -proj_mat2(1,3));
        x = A.colPivHouseholderQr().solve(b);
        return x;
    }

    void CameraManager::fullBA(int iterations, bool verbose){ 

        typedef g2o::BlockSolver< g2o::BlockSolverTraits<3,3> > Block;
        Block::LinearSolverType* linearSolver = new g2o::LinearSolverEigen<Block::PoseMatrixType>();
        Block* solver_ptr = new Block( std::unique_ptr<Block::LinearSolverType>(linearSolver) );
        g2o::OptimizationAlgorithmLevenberg* algorithm = new g2o::OptimizationAlgorithmLevenberg(std::unique_ptr<Block>(solver_ptr) );
        g2o::SparseOptimizer optimizer;

        std::vector< EdgeSE2projectXYZ* > projections_edges_vector;

        optimizer.setAlgorithm( algorithm );

        for (int i = 0; i < camera_vector_.size(); i ++){

            VertexRobotSE2* robot = new VertexRobotSE2();
            robot->setId(i);
            robot->setFixed(i==0);
            robot->setEstimate(camera_vector_[i]->robot_pose);
            robot->p_matrix = camera_vector_[i]->p_matrix;
            optimizer.addVertex(robot);             

        }

        for ( auto& allpoints: map_->map_points ){

            g2o::VertexPointXYZ* landmark = new g2o::VertexPointXYZ();
            landmark->setId(camera_vector_.size() + allpoints.first);
            landmark->setMarginalized(true);
            landmark->setEstimate(allpoints.second->p_world);
            optimizer.addVertex(landmark);

        }

        for (int i = 0; i < camera_vector_.size(); i++){

            for (auto& observations: camera_vector_[i]->observed_points){

                EdgeSE2projectXYZ* landmark_projection_edge = new EdgeSE2projectXYZ();
                landmark_projection_edge -> setVertex( 0, dynamic_cast<g2o::VertexPointXYZ*>   (optimizer.vertex(observations.second->id + camera_vector_.size())) );
                landmark_projection_edge -> setVertex( 1, dynamic_cast<VertexRobotSE2*>   (optimizer.vertex(i)));
                landmark_projection_edge -> setMeasurement (Eigen::Vector2d(camera_vector_[i]->kpts[observations.first].x,camera_vector_[i]->kpts[observations.first].y));
                landmark_projection_edge -> setRobustKernel( new g2o::RobustKernelHuber() );
                landmark_projection_edge -> setInformation( Eigen::Matrix2d::Identity() );
                optimizer.addEdge( landmark_projection_edge );
                projections_edges_vector.push_back(landmark_projection_edge);
            }

            if ( i !=0 ){

                EdgeSE2Custom* odometry_edge = new EdgeSE2Custom();
                odometry_edge -> setVertex( 0, dynamic_cast<VertexRobotSE2*>   (optimizer.vertex(i-1)));
                odometry_edge -> setVertex( 1, dynamic_cast<VertexRobotSE2*>   (optimizer.vertex(i)));
                odometry_edge -> setMeasurement (camera_vector_[i]->odom_displ);
                odometry_edge -> setRobustKernel( new g2o::RobustKernelHuber() );
                odometry_edge -> setInformation( Eigen::Matrix3d::Identity() );
                optimizer.addEdge( odometry_edge );
            }

        }
        optimizer.setVerbose(verbose);
        optimizer.initializeOptimization();
        optimizer.optimize(iterations);

        // update current estimates with optimization results
        for (int i = 0; i < camera_vector_.size(); i ++){
            VertexRobotSE2* pose_update = static_cast< VertexRobotSE2*> ( optimizer.vertex(i) );  
            camera_vector_[i]->robot_pose = pose_update->estimate();
        }
        for ( auto& allpoints: map_->map_points ){
            g2o::VertexPointXYZ* landmark_update = static_cast< g2o::VertexPointXYZ*> ( optimizer.vertex(allpoints.first + camera_vector_.size() ) ); 
            allpoints.second->p_world = landmark_update->estimate();
        }
    }

    void CameraManager::pgo(int iterations, bool verbose){ 

        typedef g2o::BlockSolver< g2o::BlockSolverTraits<3,3> > Block;
        Block::LinearSolverType* linearSolver = new g2o::LinearSolverEigen<Block::PoseMatrixType>();
        Block* solver_ptr = new Block( std::unique_ptr<Block::LinearSolverType>(linearSolver) );
        g2o::OptimizationAlgorithmLevenberg* algorithm = new g2o::OptimizationAlgorithmLevenberg(std::unique_ptr<Block>(solver_ptr) );
        g2o::SparseOptimizer optimizer;

        std::vector< EdgeSE2projectXYZ* > projections_edges_vector;

        optimizer.setAlgorithm( algorithm );

        for (int i = 0; i < camera_vector_.size(); i ++){

            VertexRobotSE2* robot = new VertexRobotSE2();
            robot->setId(i);
            robot->setFixed(i==0);
            robot->setEstimate(camera_vector_[i]->robot_pose);
            robot->p_matrix = camera_vector_[i]->p_matrix;
            optimizer.addVertex(robot);             

        }

        for ( auto& allpoints: map_->map_points ){

            g2o::VertexPointXYZ* landmark = new g2o::VertexPointXYZ();
            landmark->setId(camera_vector_.size() + allpoints.first);
            landmark->setFixed(1);
            landmark->setMarginalized(true);
            landmark->setEstimate(allpoints.second->p_world);
            optimizer.addVertex(landmark);

        }

        for (int i = 0; i < camera_vector_.size(); i++){

            for (auto& observations: camera_vector_[i]->observed_points){

                EdgeSE2projectXYZ* landmark_projection_edge = new EdgeSE2projectXYZ();
                landmark_projection_edge -> setVertex( 0, dynamic_cast<g2o::VertexPointXYZ*>   (optimizer.vertex(observations.second->id + camera_vector_.size())) );
                landmark_projection_edge -> setVertex( 1, dynamic_cast<VertexRobotSE2*>   (optimizer.vertex(i)));
                landmark_projection_edge -> setMeasurement (Eigen::Vector2d(camera_vector_[i]->kpts[observations.first].x,camera_vector_[i]->kpts[observations.first].y));
                landmark_projection_edge -> setRobustKernel( new g2o::RobustKernelHuber() );
                landmark_projection_edge -> setInformation( Eigen::Matrix2d::Identity() );
                optimizer.addEdge( landmark_projection_edge );
                projections_edges_vector.push_back(landmark_projection_edge);
            }

            if ( i !=0 ){

                EdgeSE2Custom* odometry_edge = new EdgeSE2Custom();
                odometry_edge -> setVertex( 0, dynamic_cast<VertexRobotSE2*>   (optimizer.vertex(i-1)));
                odometry_edge -> setVertex( 1, dynamic_cast<VertexRobotSE2*>   (optimizer.vertex(i)));
                odometry_edge -> setMeasurement (camera_vector_[i]->odom_displ);
                odometry_edge -> setRobustKernel( new g2o::RobustKernelHuber() );
                odometry_edge -> setInformation( Eigen::Matrix3d::Identity() );
                optimizer.addEdge( odometry_edge );
            }

        }
        optimizer.setVerbose(verbose);
        optimizer.initializeOptimization();
        optimizer.optimize(iterations);

        // update current estimates with optimization results
        for (int i = 0; i < camera_vector_.size(); i ++){
            VertexRobotSE2* pose_update = static_cast< VertexRobotSE2*> ( optimizer.vertex(i) );  
            camera_vector_[i]->robot_pose = pose_update->estimate();
        }

    }

} // visual_slam