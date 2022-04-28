#include "defs.hpp"

namespace visual_slam{

    void FrameCapture::filterMatches(
            const std::vector<std::vector<cv::DMatch> >& matches_feats,
            std::vector<cv::DMatch>* matches){
                // Clearing the current matches vector
                matches->clear();
                // Filtering matches according to the ratio test
                for (unsigned m = 0; m < matches_feats.size(); m++) {
                    if (matches_feats[m][0].distance <= matches_feats[m][1].distance * nndr_) {
                    matches->push_back(matches_feats[m][0]);
                        }
                }
            }
    
    void FrameCapture::find_matches(const cv::Mat dscs,std::vector<cv::DMatch>* matches){

        std::vector<std::vector<cv::DMatch> > matches_feats;
        // Searching the query descriptors against the features
        index_.searchDescriptors(dscs, &matches_feats, 2, 64);
        filterMatches(matches_feats,matches);
    }

    void FrameCapture::update_kptsIDs(){
        from (i = 0; i < frames_.size(); i++){
            frames_.KeypointsIDs 
            find_matches(frames_[i].KeypointsDescriptors,&frames_[i].KeypointsIDs);
        }
    }



}