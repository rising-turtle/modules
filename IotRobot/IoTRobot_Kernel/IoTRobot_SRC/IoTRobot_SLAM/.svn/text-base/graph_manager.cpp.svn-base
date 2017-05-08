/* This file is part of RGBDSLAM.
 * 
 * RGBDSLAM is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * RGBDSLAM is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with RGBDSLAM.  If not, see <http://www.gnu.org/licenses/>.
 */


#include "AIS_Lib/stuff/macros.h"
#include "AIS_Lib/math/transformation.h"
//#include <rgbdslam/CloudTransforms.h>
#include "graph_manager.h"
//#include "pcl_ros/transforms.h"
#include "pcl/io/pcd_io.h"
#include <opencv2/features2d/features2d.hpp>
#include <utility>
static unsigned short id_gen = 0;
/*
tf::Transform hogman2TF(const Transformation3 hogman_trans) {
    std::clock_t starttime=std::clock();

    tf::Transform result;
    tf::Vector3 translation;
    translation.setX(hogman_trans.translation().x());
    translation.setY(hogman_trans.translation().y());
    translation.setZ(hogman_trans.translation().z());

    tf::Quaternion rotation;
    rotation.setX(hogman_trans.rotation().x());
    rotation.setY(hogman_trans.rotation().y());
    rotation.setZ(hogman_trans.rotation().z());
    rotation.setW(hogman_trans.rotation().w());

    result.setOrigin(translation);
    result.setRotation(rotation);
    return result;
}
*/

GraphManager::GraphManager(/*GLViewer* glviewer*/) :
    freshlyOptimized_(true), //the empty graph is "optimized" i.e., sendable
//    glviewer_(glviewer),
    optimizer_(0), 
//    latest_transform_(), //constructs identity
    reset_request_(false),
    last_batch_update_(std::clock()),
    marker_id(0),
    last_matching_node_(-1),
    batch_processing_runs_(false),
	usingIMU(false),
	curr_pose(CPose3D()),
	last_pose(CPose3D())
{
    std::clock_t starttime=std::clock();

    int numLevels = 3;
    int nodeDistance = 2;

	for(int i=0;i<6;i++)
		latest_pose[i]=0;
    optimizer_ = new AIS::HCholOptimizer3D(numLevels, nodeDistance);
	bg_optimizer_ = NULL;

//    kinect_transform_.setRotation(tf::Quaternion::getIdentity());//TODO: initialize transloation too

    Max_Depth = -1;
}

GraphManager::~GraphManager() {
  //TODO: delete all Nodes
    //for (unsigned int i = 0; i < optimizer_->vertices().size(); ++i) {
    delete (optimizer_);
	if(bg_optimizer_)
		delete bg_optimizer_;

}


/// max_targets determines how many potential edges are wanted
/// max_targets < 0: No limit
/// max_targets = 0: Compare to first frame only
/// max_targets = 1: Compare to previous frame only
/// max_targets > 1: Select intelligently (TODO: rather stupid at the moment)
/*
std::vector<int> GraphManager::getPotentialEdgeTargets(const Node* new_node, int max_targets){
    int last_targets = 3; //always compare to the last n, spread evenly for the rest
	int increment;
	int counter = 0;

	max_targets -= last_targets;
	if(max_targets>0)
		increment = ceil((float)(graph_.size()- last_targets)/(float)(max_targets-last_targets));
	else
		increment = graph_.size();

    std::vector<int> ids_to_link_to;
    //Special Cases
    if(graph_.size() == 0){
        return ids_to_link_to;
    }

	map <int, Node*>::reverse_iterator graph_iter  = graph_.rbegin();
	for ( graph_iter++ ; graph_iter != graph_.rend(); graph_iter++){
		// make sure the last n is inserted into ids_to_link_to
		if((--last_targets) > 0){
			ids_to_link_to.push_back(graph_iter->first);
		}
		else{// the rest spread evenly!
			if(0 == (++counter)%increment)
				ids_to_link_to.push_back(graph_iter->first);
		}
	}
    return ids_to_link_to;
}
*/
//std::vector<int> GraphManager::getPotentialEdgeTargets(const Node* new_node, int max_targets){
//
//	std::vector<int> ids_to_link_to;
//
//	// the previous frame has been computed!!!
//	max_targets--;
//	//Special Cases
//	if(graph_.size() == 0){
//		return ids_to_link_to;
//	}else if(graph_.size()-1 > max_targets){
//		// recent max_targets-1 frames and the original frame
//		map <int, Node*>::reverse_iterator graph_iter  = graph_.rbegin();
//		for ( graph_iter++ ; (graph_iter != graph_.rend()) && max_targets>0; graph_iter++, max_targets--)
//			ids_to_link_to.push_back(graph_iter->first);
//		// insert the original frame
//		ids_to_link_to.push_back(0);
//	}else{
//		// all the frames
//		map <int, Node*>::reverse_iterator graph_iter  = graph_.rbegin();
//		for ( graph_iter++ ; (graph_iter != graph_.rend()); graph_iter++)
//			ids_to_link_to.push_back(graph_iter->first);
//	}
//	return ids_to_link_to;
//}
/// max_targets determines how many potential edges are wanted
/// max_targets < 0: No limit
/// max_targets = 0: Compare to first frame only
/// max_targets = 1: Compare to previous frame only
/// max_targets > 1: Select intelligently (TODO: rather stupid at the moment)
std::vector<int> GraphManager::getPotentialEdgeTargets(const Node* new_node, int max_targets){
	int last_targets = 3; //always compare to the last n, spread evenly for the rest
	int increment;
	int counter = 0;

	max_targets -= last_targets;
	if(max_targets>0)
		increment = ceil((float)(graph_.size()- last_targets)/(float)(max_targets-last_targets));
	else
		increment = graph_.size();

	std::vector<int> ids_to_link_to;

	//Special Cases
	if(graph_.size() == 0){
		return ids_to_link_to;
	}

	map <int, Node*>::reverse_iterator graph_iter  = graph_.rbegin();
	for ( graph_iter++ ; graph_iter != graph_.rend(); graph_iter++){
		// make sure the last n is inserted into ids_to_link_to
		if((--last_targets) > 0){
			ids_to_link_to.push_back(graph_iter->first);
		}
		else{// the rest spread evenly!
			if(0 == (++counter)%increment)
				ids_to_link_to.push_back(graph_iter->first);
		}
	}
	return ids_to_link_to;
}
void GraphManager::resetGraph(){
    int numLevels = 3;
    int nodeDistance = 2;
    marker_id =0;
//    time_of_last_transform_= ros::Time();
//    last_batch_update_=std::clock();
    delete optimizer_; 
	if(bg_optimizer_)
		delete bg_optimizer_;
    optimizer_ = new AIS::HCholOptimizer3D(numLevels, nodeDistance);

	//clear all nodes! also delete the nodes
	for(std::map<int, Node*>::iterator it=graph_.begin();
		it!=graph_.end(); it++)
		delete it->second;
	graph_.clear();

    freshlyOptimized_= false;
    reset_request_ = false;
	bg_optimizer_ = NULL;
	id_gen = 0;
	// reset the last pose!
	for(int i=0;i<6;i++)
		latest_pose[i]=0;
	// reset the pose!
	last_pose = CPose3D();
	curr_pose = CPose3D();
}

// returns true, iff node could be added to the cloud
bool GraphManager::addNode(Node* new_node) {

    last_inlier_matches_.clear();
    if(reset_request_) resetGraph(); 

    if (new_node->feature_locations_2d_.size() <= 50){
        return false;
    }

	// Initialized as unmatched
	matched = false;
    //set the node id only if the node is actually added to the graph
    //needs to be done here as the graph size can change inside this function
	new_node->id_ = id_gen;//graph_.size();
	id_gen++;

    //First Node, so only build its index, insert into storage and add a
    //vertex at the origin, of which the position is very certain
    if (graph_.size()==0){
        new_node->buildFlannIndex(); // create index so that next nodes can use it
        graph_[new_node->id_] = new_node;

		// convert pose data format from CPose to Transformation3
		Eigen::Matrix4f eigen_mat;
		last_pose.getHomogeneousMatrix(eigen_mat);
		Eigen::Affine3f eigen_transform(eigen_mat);
		Eigen::Quaternionf eigen_quat(eigen_transform.rotation());
		Vector3 translation(eigen_mat(0, 3), eigen_mat(1, 3), eigen_mat(2, 3));
		Quaternion rotation(eigen_quat.x(), eigen_quat.y(), eigen_quat.z(), eigen_quat.w());
		Transformation3 result(translation, rotation);

        optimizer_->addVertex(new_node->id_, result, 1e9*Matrix6::eye(1.0)); //fix at origin
		pointcloud_type const * the_pc(&(new_node->pc_col));
		return true;
    }

    unsigned int num_edges_before = optimizer_->edges().size(); 
    marker_id = 0; //overdraw old markers
    last_matching_node_ = -1;


    //MAIN LOOP: Compare node pairs ######################################################################
    //First check if trafo to last frame is big
    Node* prev_frame = graph_.rbegin()->second;

	MatchingResult mr;
	if(!usingIMU)
		mr = new_node->matchNodePair(prev_frame);
	else
		mr = new_node->matchLastNode(prev_frame,last_pose,curr_pose);

	// for saving images
	lastmr = mr;
	unsigned int nAddedEdge = 0;
    if(mr.edge.id1 >= 0 && !isBigTrafo(mr.edge.mean)){
		matched = true; 
        return false;
    } else if(mr.edge.id1 >= 0){
		//if (isNoiseTrafo(mr.edge.mean))
			//return false;
        if (addEdgeToHogman(mr.edge, true)) {
            last_matching_node_ = mr.edge.id1;
            last_inlier_matches_ = mr.inlier_matches;
            last_matches_ = mr.all_matches;
			nAddedEdge++;
        }
	}

	std::vector<int> vertices_to_comp = getPotentialEdgeTargets(new_node, global_potential_nodes); //vernetzungsgrad
//	for (int id_of_id = 0;  id_of_id<vertices_to_comp.size(); id_of_id++)
		//cout<<":"<<vertices_to_comp[id_of_id];
//	cout<<endl;
	
	for (int id_of_id = (int)vertices_to_comp.size()-1; (id_of_id >=0 && nAddedEdge<global_connectivity);id_of_id--){ 
		Node* abcd = graph_[vertices_to_comp[id_of_id]];
        MatchingResult mr = new_node->matchNodePair(abcd);
        if(mr.edge.id1 >= 0){
            if (addEdgeToHogman(mr.edge, isBigTrafo(mr.edge.mean))) { //TODO: result isBigTrafo is not considered
                last_matching_node_ = mr.edge.id1;
                last_inlier_matches_ = mr.inlier_matches;
                last_matches_ = mr.all_matches;
				nAddedEdge++;
            }
        }
    }
    //END OF MAIN LOOP: Compare node pairs ######################################################################

    if (optimizer_->edges().size() > num_edges_before) { //Success
        //double me_start_t = ::GetTickCount();
		new_node->buildFlannIndex();
        graph_[new_node->id_] = new_node;
		
		PF_START(gl_pf_start);
        optimizeGraph();
		PF_FETCH(gl_pf_me, gl_pf_start);

		//FORCELY add current node into bg_optimizer if NOT added.
		if(bg_optimizer_ && !(bg_optimizer_->vertex(new_node->id_))){
			AISNavigation::PoseGraph3D::Vertex* v1 = optimizer_->vertex(prev_frame->id_);
			AISNavigation::PoseGraph3D::Vertex* v2 = optimizer_->vertex(new_node->id_);
			AISNavigation::PoseGraph3D::Vertex* bg_v1 = bg_optimizer_->vertex(prev_frame->id_);
			AISNavigation::PoseGraph3D::Vertex* bg_v2 = bg_optimizer_->addVertex(new_node->id_, Transformation3(), Matrix6::eye(1.0));
			bg_optimizer_->addEdge(bg_v1, bg_v2, ((v1->transformation.inverse())*v2->transformation), Matrix6::eye(360.0));
		}

		//Get the 6D pose of the new node
		AISNavigation::PoseGraph3D::Vertex* v = optimizer_->vertex(new_node->id_);
		latest_pose[0] = v->transformation.translation().x();
		latest_pose[1] = v->transformation.translation().y();
		latest_pose[2] = v->transformation.translation().z();

		_Vector<3, double> rpy = v->transformation.rotation().rotationMatrix().angles();
		latest_pose[3] = rpy.roll();
		latest_pose[4] = rpy.pitch();
		latest_pose[5] = rpy.yaw();

	//	cout<<"after optimizer!"<<endl;
	//	cout<<"(x,y,z)" <<"("<<latest_pose[0]<<","<<latest_pose[1]<<","<<latest_pose[2]<<")" \
	//			<<"(r,p,y)" <<"("<<R2D(latest_pose[3])<<","<<R2D(latest_pose[4])<<","<<R2D(latest_pose[5])<<")"<<endl;

		// check the size of optimizer, create bg_optimizer if needed. switch bg_optimizer to active optimizer if size of bg_optimizer reach the threshold.
		if(optimizer_->vertices().size() > global_graph_size){
			//cout<<"Now in size> global_graph_size!"<<endl;
			// create bg_optimizer
			if(bg_optimizer_ == NULL){
				/*
				bg_optimizer_ = new AIS::HCholOptimizer3D(3, 2);
				// add current node as the root node!
				// keep the root node for ever!
				AISNavigation::PoseGraph3D::Vertex* root_v = bg_optimizer_->addVertex(0, Transformation3(), 1e9*Matrix6::eye(1.0));
				AISNavigation::PoseGraph3D::Vertex* current_v = bg_optimizer_->addVertex(new_node->id_, Transformation3(), Matrix6::eye(1.0));
				bg_optimizer_->addEdge(current_v, root_v, v->transformation, Matrix6::eye(360.0));
				bg_optimizer_->optimize(10,true);
				*/
				bg_optimizer_ = new AIS::HCholOptimizer3D(3, 2);
				// add current node as the root node!
				// keep the root node for ever!
				//AISNavigation::PoseGraph3D::Vertex* root_v = bg_optimizer_->addVertex(0, Transformation3(), 1e9*Matrix6::eye(1.0));
				AISNavigation::PoseGraph3D::Vertex* current_v = bg_optimizer_->addVertex(new_node->id_, v->transformation, Matrix6::eye(1.0));
				//bg_optimizer_->addEdge(current_v, root_v, v->transformation, Matrix6::eye(360.0));
				//bg_optimizer_->optimize(10,true);

				cout<<"Switch to background Opti"<<endl;
			}
			// switch bg_optimizer to active optimizer if graph size reach the threshold
			else if(bg_optimizer_->vertices().size() > global_bg_graph_threshold){
				//1. delete the current active optimizer!
				delete optimizer_;
				//2. remove the unreferenced node in graph manager
				for(std::map<int, Node* >::iterator graph_it = graph_.begin(); graph_it != graph_.end();){
					// ONLY remove the node NOT used by bg_optimizer.
					if(bg_optimizer_->vertices().find(graph_it->first) == bg_optimizer_->vertices().end()){
						Node* node_to_del = graph_it->second;
						graph_it++;
						graph_.erase(node_to_del->id_);
						delete node_to_del;
					}else
						graph_it++;
				}
				//3. set bg_optimizer_ to optimizer_
				optimizer_ = bg_optimizer_;
				bg_optimizer_ = NULL;
				cout<<"Switch to active Opti"<<endl;
			}
		}
	//	cout<<"return to testsurf!"<<endl;
		return true;
	}else{
			return false;
	}
}


///Get the norm of the translational part of an affine matrix (Helper for isBigTrafo)
void GraphManager::mat2dist(const Eigen::Matrix4f& t, double &dist){
    dist = sqrt(t(0,3)*t(0,3)+t(1,3)*t(1,3)+t(2,3)*t(2,3));
}
///Get euler angles from affine matrix (helper for isBigTrafo)
void GraphManager::mat2RPY(const Eigen::Matrix4f& t, double& roll, double& pitch, double& yaw) {
    roll = atan2(t(2,1),t(2,2));
    pitch = atan2(-t(2,0),sqrt(t(2,1)*t(2,1)+t(2,2)*t(2,2)));
    yaw = atan2(t(1,0),t(0,0));
}
// true iff edge qualifies for generating a new vertex
bool GraphManager::isBigTrafo(const Eigen::Matrix4f& t){
    double roll, pitch, yaw, dist;

    mat2RPY(t, roll,pitch,yaw);
    mat2dist(t, dist);

    roll = roll/M_PI*180;
    pitch = pitch/M_PI*180;
    yaw = yaw/M_PI*180;

    double max_angle = max(roll,max(pitch,yaw));

    // at least 10cm or 5deg
    return (dist > global_min_translation_meter || max_angle > global_min_rotation_degree);
}

bool GraphManager::isBigTrafo(const Transformation3& t){
    float angle_around_axis = 2.0*acos(t._rotation.w()) *180.0 / M_PI;
    float dist = t._translation.norm();
    //QString infostring;
    //ROS_INFO("Rotation:% 4.2f, Distance: % 4.3fm", angle_around_axis, dist);
    //infostring.sprintf("Rotation:% 4.2f, Distance: % 4.3fm", angle_around_axis, dist);
    //Q_EMIT setGUIInfo2(infostring);
    return (dist > global_min_translation_meter || angle_around_axis > global_min_rotation_degree);
}

// true iff edge qualifies for generating a new vertex
bool GraphManager::isNoiseTrafo(const Eigen::Matrix4f& t){
	// the big traffic MAYBE nosie traffic!!
	double dist;
	mat2dist(t, dist);
	// at most 2M
	return (dist > global_max_translation_meter);
}

bool GraphManager::isNoiseTrafo(const Transformation3& t){
	// the big traffic MAYBE nosie traffic!!
	float dist = t._translation.norm();
	// at most 2M
	return (dist > global_max_translation_meter);
}

bool GraphManager::addEdgeToHogman(AIS::LoadedEdge3D edge, bool largeEdge) {

	freshlyOptimized_ = false;
	AIS::PoseGraph3D::Vertex* v1 = optimizer_->vertex(edge.id1);
	AIS::PoseGraph3D::Vertex* v2 = optimizer_->vertex(edge.id2);

	// at least one vertex has to be created, assert that the transformation
	// is large enough to avoid to many vertices on the same spot
	if (!v1 || !v2){
		if (!largeEdge) {
			return false; 
		}
	}
	if (!v1 && !v2)
		return false;

	if (!v1) {
		v1 = optimizer_->addVertex(edge.id1, Transformation3(), Matrix6::eye(1.0));
		assert(v1);
	}
	if (!v2) {
		v2 = optimizer_->addVertex(edge.id2, Transformation3(), Matrix6::eye(1.0));
		assert(v2);
	}
	optimizer_->addEdge(v1, v2, edge.mean, edge.informationMatrix);

	// bg_optimizer_ is active if not NULL.
	if(bg_optimizer_){
		
		/*
		std::map<int, AISNavigation::Graph::Vertex*>::iterator vertex_iter = bg_optimizer_->vertices().begin();
		vertex_iter++;
		if (vertex_iter->first > ((edge.id1<edge.id2)?edge.id1:edge.id2))
			return true;
		*/

		AIS::PoseGraph3D::Vertex* bg_v1 = bg_optimizer_->vertex(edge.id1);
		AIS::PoseGraph3D::Vertex* bg_v2 = bg_optimizer_->vertex(edge.id2);

		// make sure the pre node is contained in bg_optimizer_;
		if((edge.id1<edge.id2 && !bg_v1) || edge.id1>edge.id2 && !bg_v2)
			return true;

		if(bg_v1 || bg_v2){
			if(!bg_v1)
				bg_v1 = bg_optimizer_->addVertex(edge.id1, Transformation3(), Matrix6::eye(1.0));
			if(!bg_v2)
				bg_v2 = bg_optimizer_->addVertex(edge.id2, Transformation3(), Matrix6::eye(1.0));

			bg_optimizer_->addEdge(bg_v1, bg_v2, edge.mean, edge.informationMatrix);
		}
	}
	return true;
}

void GraphManager::optimizeGraph(bool online){
	//cout<<"INTO OG: "<<graph_.size()<<"=="<<optimizer_->vertices().size()<<":"<<optimizer_->edges().size()<<endl;
    std::clock_t starttime=std::clock();
    const int iterations = 10;
    int currentIt = optimizer_->optimize(iterations, online);
	if(bg_optimizer_){
	//	cout<<"INTTO OG_BG: "<<bg_optimizer_->vertices().size()<<":"<<bg_optimizer_->edges().size()<<endl;
		bg_optimizer_->optimize(iterations, online);
	//	cout<<"OUTOF OG_BG"<<endl;
	}
	//cout<<"OUTOF OG"<<endl;
    //ROS_INFO_STREAM("Hogman Statistics: " << optimizer_->vertices().size() << " nodes, " 
    //                << optimizer_->edges().size() << " edges. "
    //                << "chi2: " << optimizer_->chi2()
    //                << ", Iterations: " << currentIt);

	freshlyOptimized_ = true;
	//if(optimizer_->vertices().size()>0){
	//	AISNavigation::PoseGraph3D::Vertex* v = optimizer_->vertex(optimizer_->vertices().size()-1);
	//	latest_pose[0] = v->transformation.translation().x();
	//	latest_pose[1] = v->transformation.translation().y();
	//	latest_pose[2] = v->transformation.translation().z();

	//	_Vector<3, double> rpy = v->transformation.rotation().rotationMatrix().angles();
	//	latest_pose[3] = rpy.roll();;
	//	latest_pose[4] = rpy.pitch();
	//	latest_pose[5] = rpy.yaw();
	//}



	/*if(optimizer_->vertices().size()>0){
		AISNavigation::PoseGraph3D::Vertex* v = optimizer_->vertex(optimizer_->vertices().size()-1);
		latest_pose[0] = v->transformation.translation().x();
		latest_pose[1] = v->transformation.translation().y();
		latest_pose[2] = v->transformation.translation().z();
		latest_pose[3] = v->transformation.rotation().roll();
		latest_pose[4] = v->transformation.rotation().pitch();
		latest_pose[5] = v->transformation.rotation().yaw();
	}*/
    //kinect_transform_ =  hogman2TF(v->transformation);
    //pcl_ros::transformAsMatrix(kinect_transform_, latest_transform_);
    //latest_transform_ = hogman2QMatrix(v->transformation); 

    /*publish the corrected transforms to the visualization module every five seconds
    if( ((std::clock()-last_batch_update_) / (double)CLOCKS_PER_SEC) > 2){
        publishCorrectedTransforms();
        last_batch_update_ = std::clock();
    }*/
//    ROS_INFO_STREAM_COND_NAMED(( (std::clock()-starttime) / (double)CLOCKS_PER_SEC) > global_min_time_reported, "timings", "function runtime: "<< ( std::clock() - starttime ) / (double)CLOCKS_PER_SEC  <<"sec"); 
}

void GraphManager::reset(){
    reset_request_ = true;
}

void GraphManager::deleteLastFrame(){
    if(graph_.size() <= 1) {
//      ROS_INFO("Resetting, as the only node is to be deleted");
      reset_request_ = true;
//      Q_EMIT deleteLastNode();
      return;
    }
   // AISNavigation::PoseGraph3D::Vertex* v_to_del = optimizer_->vertex(optimizer_->vertices().size()-1);//last vertex
   AISNavigation::PoseGraph3D::Vertex* v_to_del = optimizer_->vertex(optimizer_->vertices().rbegin()->first);//last vertex
/*
    AISNavigation::PoseGraph3D::Vertex *v1, *v2; //used in loop as temporaries
    AISNavigation::PoseGraph3D::EdgeSet::iterator edge_iter = optimizer_->edges().begin();
    for(;edge_iter != optimizer_->edges().end(); edge_iter++) {
        v1 = static_cast<AISNavigation::PoseGraph3D::Vertex*>((*edge_iter)->from());
        v2 = static_cast<AISNavigation::PoseGraph3D::Vertex*>((*edge_iter)->to());
        if(v1->id() == v_to_del->id() || v2->id() == v_to_del->id()) 
          optimizer_->removeEdge((*edge_iter));
    }
*/
	int id_to_del = v_to_del->id();

    optimizer_->removeVertex(v_to_del);
	optimizer_->optimize(10, true);

	if(bg_optimizer_){
		v_to_del = bg_optimizer_->vertex(id_to_del);
		if(v_to_del){
			bg_optimizer_->removeVertex(v_to_del);
			bg_optimizer_->optimize(10, true);
		}
	}

    //graph_.erase(graph_.size()-1);
	graph_.erase(id_to_del);
    //optimizeGraph(false);//s.t. the effect of the removed edge transforms are removed to
}

// remove the vertex and edges in optimizer (also release memory space used in optimizer).
// remove the vertex in graphman, but the node pointer should be released manaully!!!
void GraphManager::eraseNode(Node* node){
    if(graph_.size() <= 1) {
//      ROS_INFO("Resetting, as the only node is to be deleted");
      reset_request_ = true;
//      Q_EMIT deleteLastNode();
      return;
    }
	// find the corresponding vertex in optimizor
	AISNavigation::PoseGraph3D::Vertex* v_to_del = optimizer_->vertex(node->id_);
	// the related edges can be removed by removeVertex automatically! 
	// the memory space is also released automatically!
	optimizer_->removeVertex(v_to_del);
	graph_.erase(node->id_);
    optimizeGraph();
}
// Delete the nodes out of range. The range is defined as a sphere.
// The pointers in graphman will also be released automatically!!!
void GraphManager::deleteOutRangeFrames(double x, double y, double z, double radius){
    if(graph_.size() <= 1) {
//      ROS_INFO("Resetting, as the only node is to be deleted");
      reset_request_ = true;
//      Q_EMIT deleteLastNode();
      return;
    }
	std::vector<int> vector_to_del;

	// 1. check all the vertices to find out the nodes out of range
	// 2. remove all the vertices out of range and the related edge at the same time!
	std::map<int, AISNavigation::Graph::Vertex*>::iterator vertex_iter = optimizer_->vertices().begin();
	for(; vertex_iter!= optimizer_->vertices().end();){
		AISNavigation::PoseGraph3D::Vertex* v_to_del =  reinterpret_cast<AISNavigation::PoseGraph3D::Vertex*>(vertex_iter->second);
		double x1=v_to_del->transformation.translation().x();
		double y1=v_to_del->transformation.translation().y();
		double z1=v_to_del->transformation.translation().z();

		if( (x1-x)*(x1-x)+(y1-y)*(y1-y)+(z1-z)*(z1-z) > radius*radius){
			vector_to_del.push_back(vertex_iter->first);
			// First pointer to the next vertex!
			vertex_iter++;
			// Now remove current vertex safely!
			optimizer_->removeVertex(v_to_del);
			cout<<"Delete out range node"<<endl;
		}else
			vertex_iter++;
	}
	// 3. remove all the vertices out of range in graphmanager graph
	for(int i=0; i<vector_to_del.size(); i++){
		Node* node_to_del=graph_[vector_to_del[i]];
		graph_.erase(node_to_del->id_);
		delete node_to_del;
	}
}






void GraphManager::setMaxDepth(float max_depth){
	Max_Depth = max_depth;
}

void GraphManager::resetSession(CPose3D initPose){
	unsigned short preid = id_gen;
	resetGraph();
	id_gen = preid;
	last_pose = initPose;
}


//From: /opt/ros/unstable/stacks/perception_pcl/pcl/src/pcl/registration/transforms.hpp
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief Apply an affine transform defined by an Eigen Transform
  * \param cloud_in the input point cloud
  * \param cloud_to_append_to the transformed cloud will be appended to this one
  * \param transform a tf::Transform stating the transformation of cloud_to_append_to relative to cloud_in
  * \note The density of the point cloud is lost, since density implies that the origin is the point of view
  * \note Can not(?) be used with cloud_in equal to cloud_to_append_to
  */
//template <typename PointT> void
//transformAndAppendPointCloud (const pcl::PointCloud<PointT> &cloud_in, pcl::PointCloud<PointT> &cloud_to_append_to,
//                              const tf::Transform transformation)
/*
void transformAndAppendPointCloud (const pointcloud_type &cloud_in, 
                                   pointcloud_type &cloud_to_append_to,
                                   const tf::Transform transformation, float Max_Depth)
{
    bool compact = !global_preserve_raster_on_save;
    Eigen::Matrix4f eigen_transform;
    pcl_ros::transformAsMatrix(transformation, eigen_transform);
    unsigned int cloud_to_append_to_original_size = cloud_to_append_to.size();
    if(cloud_to_append_to.points.size() ==0){
        cloud_to_append_to.header   = cloud_in.header;
        cloud_to_append_to.width    = 0;
        cloud_to_append_to.height   = 0;
        cloud_to_append_to.is_dense = false;
    }

    //ROS_INFO("Max_Depth = %f", Max_Depth);
    //ROS_INFO("cloud_to_append_to_original_size = %i", cloud_to_append_to_original_size);

    //Append all points untransformed
    cloud_to_append_to += cloud_in;

    Eigen::Matrix3f rot   = eigen_transform.block<3, 3> (0, 0);
    Eigen::Vector3f trans = eigen_transform.block<3, 1> (0, 3);
    point_type origin = point_type();
    origin.x = 0;
    origin.y = 0;
    origin.z = 0;
    int j = 0;
    for (size_t i = 0; i < cloud_in.points.size (); ++i)
    { 
     Eigen::Map<Eigen::Vector3f> p_in (&cloud_in.points[i].x, 3, 1);
     Eigen::Map<Eigen::Vector3f> p_out (&cloud_to_append_to.points[j+cloud_to_append_to_original_size].x, 3, 1);
     if(compact){ cloud_to_append_to.points[j+cloud_to_append_to_original_size] = cloud_in.points[i]; }
     //filter out points with a range greater than the given Parameter or do nothing if negativ
     if(Max_Depth >= 0){
		 if(pcl::squaredEuclideanDistance(cloud_in.points[i], origin) > Max_Depth*Max_Depth){
			p_out[0]= numeric_limits<float>::quiet_NaN();
			p_out[1]= numeric_limits<float>::quiet_NaN();
			p_out[2]= numeric_limits<float>::quiet_NaN();
			if(!compact) j++; 
			continue;
		  }
      }
      if (pcl_isnan (cloud_in.points[i].x) || pcl_isnan (cloud_in.points[i].y) || pcl_isnan (cloud_in.points[i].z)){
        if(!compact) j++;
    	  continue;
      }
      p_out = rot * p_in + trans;
      j++;
    }
    if(compact){
      cloud_to_append_to.points.resize(j+cloud_to_append_to_original_size);
      cloud_to_append_to.width    = 1;
      cloud_to_append_to.height   = j+cloud_to_append_to_original_size;
	}
}
*/