#include "FlirterNode.h"
#include <feature/Detector.h>
#include <feature/ShapeContext.h>
#include <feature/BetaGrid.h>
#include <feature/RangeDetector.h>
#include <feature/CurvatureDetector.h>
#include <feature/NormalBlobDetector.h>
#include <feature/NormalEdgeDetector.h>
#include <feature/RansacFeatureSetMatcher.h>
#include <feature/RansacMultiFeatureSetMatcher.h>
#include <sensorstream/CarmenLog.h>
#include <sensorstream/LogSensorStream.h>
#include <sensorstream/SensorStream.h>
#include <utils/SimpleMinMaxPeakFinder.h>
#include <utils/HistogramDistances.h>

const int g_match_threshold = 15;

bool CFliterNode::g_IsInit = false;
SimpleMinMaxPeakFinder* CFliterNode::g_peakMinMax = NULL;

// detectors
string CFliterNode::g_detector_name = "";
Detector* CFliterNode::g_detector = NULL;
CurvatureDetector * CFliterNode::g_detectorCurvature = NULL;
NormalBlobDetector * CFliterNode::g_detectorNormalBlob = NULL;
NormalEdgeDetector* CFliterNode::g_detectorNormalEdge = NULL;
RangeDetector* CFliterNode::g_detectorRange = NULL;

//distance
string CFliterNode::g_distance_name="";
HistogramDistance<double> * CFliterNode::g_dist = NULL;

// descriptor
string CFliterNode::g_descriptor_name = "";
ShapeContextGenerator* CFliterNode::g_shapeGenerator=NULL;
DescriptorGenerator* CFliterNode::g_descriptor =NULL;
BetaGridGenerator* CFliterNode::g_betaGenerator = NULL;

// match strategy
RansacFeatureSetMatcher* CFliterNode::g_ransac = NULL;

CFliterNode::CFliterNode(PMScan* pmscan, std::string laser_name):
m_pLaserRead(NULL),                                                                                 
m_pFMatch(new CPolarMatch(laser_name)), 
m_pScan(NULL)
{
	if(m_pFMatch == NULL){
		std::cerr<<"Worng Laser: "<<laser_name<<std::endl;
		throw 1;
	}
	if(!CFliterNode::g_IsInit)
	{
		InitFliter();
		g_IsInit = true;
	}
	if(pmscan == NULL){
		std::cerr<<"pmscan is NULL!"<<std::endl;
		throw 1;
	}
	// these actually a duplicated value for the laser scans
	m_pScan = new PMScan(*pmscan);
	m_pLaserRead = fromPM2LR(pmscan);
	if(m_pLaserRead == NULL){
		std::cout<<"lread is NULL!"<<std::endl;
		throw 1;
	}
	InitNode(m_pLaserRead);	
}

CFliterNode::CFliterNode(LaserReading* lread, string laser_name):
m_pLaserRead(NULL),
m_pScan(NULL),
m_pFMatch(new CPolarMatch(laser_name))
{
	if(m_pFMatch == NULL){
		std::cerr<<"Wrong Laser: "<<laser_name<<std::endl;
		throw 1;
	}
	if(!CFliterNode::g_IsInit)
	{
		InitFliter();
		g_IsInit = true;
	}
	if(lread == NULL){
		std::cout<<"lread is NULL!"<<std::endl;
		throw 1;
	}
	// this is dangerous for deleting lread outside
	m_pLaserRead = new LaserReading(*lread);
	m_pScan = fromLR2PM(m_pLaserRead);
	if(m_pScan == NULL){
		std::cout<<"Scan is NULL!"<<std::endl;
		// throw 1;
	}
	InitNode(m_pLaserRead);
	assert(m_featurePoints.size()==m_featurePointsLocal.size());
}
CFliterNode::~CFliterNode(){
	UnitNode();
}
void CFliterNode::InitNode(LaserReading* lread)
{
	// detect feature points
	g_detector->detect(*lread,m_featurePoints);
	m_pose = lread->getLaserPose();
	// describe feature points
	for(int i=0;i<m_featurePoints.size();i++)
		m_featurePoints[i]->setDescriptor(g_descriptor->describe(*m_featurePoints[i],*lread));

	m_featurePointsLocal.resize(m_featurePoints.size(),NULL);
	// calculate local feature points
	for(int i=0;i<m_featurePoints.size();i++)
	{
		InterestPoint* plocal = new InterestPoint(*m_featurePoints[i]);
		plocal->setPosition(m_pose.ominus(plocal->getPosition()));
		m_featurePointsLocal[i] = plocal;
	}
	return ;
}
void CFliterNode::UnitNode()
{
	if(m_pScan!=NULL)
		delete m_pScan;
	if(m_pLaserRead!=NULL)
		delete m_pLaserRead;
	for(int i=0;i<m_featurePoints.size();i++){
		if(m_featurePoints[i]!=NULL)
		{
			delete m_featurePoints[i];
			m_featurePoints[i]=NULL;
		}
	}
	for(int i=0;i<m_featurePointsLocal.size();i++){
		if(m_featurePointsLocal[i]!=NULL)
		{
			delete m_featurePointsLocal[i];
			m_featurePointsLocal[i]=NULL;
		}
	}
}

bool CFliterNode::matchNodePairLocal( CFliterNode* refNode, OrientedPoint2D& transform)
{
	if(refNode ==NULL){
			return false;
	}	
	//static int match_threshold = 15; // actually it is added by feel
	int ret = matchFeaturePoints(refNode->m_featurePointsLocal,m_featurePointsLocal,transform);
	if(ret >= g_match_threshold)
	{
		//this->m_pose = refNode->m_pose + (transform);
		this->m_relpose = transform;
		return true;
	}
	else
	{
		//this->m_relpose = m_pose - (refNode->m_pose);
		this->m_relpose = refNode->m_pose.ominus(m_pose);
		transform = this->m_relpose;
	}
	return false;
}
bool CFliterNode::matchNodePairGlobal( CFliterNode* refNode, OrientedPoint2D& transform)
{
	if(refNode == NULL)
		return false;

	int ret= matchFeaturePoints(refNode->m_featurePoints,m_featurePointsLocal,transform);
	if(ret>=g_match_threshold)	// successfully matched
	{	
		//this->m_pose = transform;
		this->m_relpose = transform - refNode->m_pose;
		return true;
	}
	else	// failed to match
	{
		this->m_relpose = m_pose - refNode->m_pose;
	}
	//this->m_relpose = m_pose - (refNode->m_pose);
	transform = this->m_relpose;
	return false;
}

int CFliterNode::matchFeaturePoints(std::vector<InterestPoint*>& fpref,std::vector<InterestPoint*>& fpcur,
								OrientedPoint2D& transform)
{
	std::vector< std::pair<InterestPoint*, InterestPoint* > > correspondences;
	double result;
	result=g_ransac->matchSets(fpref, fpcur, transform, correspondences);
	int n_inliers = correspondences.size();

	if(correspondences.size()<3)
	{
		cout<<"correspondences are too few!"<<endl;
		return n_inliers;
	}
	if(result >=1e17){
		cout<<"result error is too big!"<<endl;
		return 0;
	}
	return n_inliers;
}

// match previous node using PSM or ICP
bool CFliterNode::matchNodeFrontend(CFliterNode* pref)
{
	float err = m_pFMatch->FMatch(pref->m_pScan,m_pScan);
	if(err<0){
		std::cout<<"Failed in FMatch!"<<std::endl;
		return false;
	}
	m_relpose = OrientedPoint2D(m_pScan->rx,m_pScan->ry,m_pScan->th);
	return true;
}

void CFliterNode::resetInit(){
	CFliterNode::g_IsInit =false;
}

void CFliterNode::InitFliter(){
	CFliterNode::g_peakMinMax = new SimpleMinMaxPeakFinder(getParameter().minPeak, getParameter().minPeakDistance);
	// set detectorType
	switch(getParameter().detectorType){
		case 0:
			g_detectorCurvature = new CurvatureDetector(g_peakMinMax,getParameter().scale, 
				getParameter().baseSigma,getParameter().sigmaStep,getParameter().dmst);
			g_detectorCurvature->setUseMaxRange(getParameter().useMaxRange);
			g_detector = g_detectorCurvature;
			g_detector_name = "curvature";
			break;
		case 1:
			g_detectorNormalEdge = new NormalEdgeDetector(g_peakMinMax,getParameter().scale,getParameter().baseSigma, 
				getParameter().sigmaStep,getParameter().window);
			g_detectorNormalEdge->setUseMaxRange(getParameter().useMaxRange);
			g_detector = g_detectorNormalEdge;
			g_detector_name = "edge";
			break;
		case 2:
			g_detectorNormalBlob = new NormalBlobDetector(g_peakMinMax,getParameter().scale,getParameter().baseSigma, 
				getParameter().sigmaStep,getParameter().window);
			g_detectorNormalBlob->setUseMaxRange(getParameter().useMaxRange);
			g_detector = g_detectorNormalBlob;
			g_detector_name = "blob";
			break;
		case 3:
			g_detectorRange = new RangeDetector(g_peakMinMax,getParameter().scale,getParameter().baseSigma,getParameter().sigmaStep);
			g_detectorRange->setUseMaxRange(getParameter().useMaxRange);
			g_detector = g_detectorRange;
			g_detector_name = "range";
			break;
		default:
			cerr<<"wrong detecor type!"<<endl;
			exit(-1);
	}
	// set distance types
	switch(getParameter().distanceType){
	case 0:
		g_dist =  new EuclideanDistance<double>(); 
		g_distance_name = "euclid";
		break;
	case 1:
		g_dist = new Chi2Distance<double>();
		g_distance_name = "chi2";
		break;
	case 2:
		g_dist = new SymmetricChi2Distance<double>();
		g_distance_name = "symchi2";
		break;
	case 3:
		g_dist = new BatthacharyyaDistance<double>();
		g_distance_name = "batt";
		break;
	case 4:
		g_dist = new KullbackLeiblerDistance<double>();
		g_distance_name = "kld";
		break;
	case 5:
		g_dist = new JensenShannonDistance<double>();
		g_distance_name = "jsd";
	default:
		cerr<<"wrong distance type!"<<endl;
		exit(-1);
	}
	// set descriptors 
	switch(getParameter().descriptorType){
	case 0:
		g_betaGenerator =  new BetaGridGenerator(0.02, 0.5, 4, 12);
		g_betaGenerator->setDistanceFunction(g_dist);
		g_descriptor = g_betaGenerator;
		g_descriptor_name = "beta";
		break;
	case 1:
		g_shapeGenerator = new  ShapeContextGenerator(0.02, 0.5, 4, 12);
		g_shapeGenerator->setDistanceFunction(g_dist);
		g_descriptor = g_shapeGenerator;
		g_descriptor_name = "shape";
		break;
	default:
		cerr<<"wrong descriptor type!"<<endl;
		exit(-1);
	}
	// set match strategy
	switch(getParameter().matchStrategy){
		case 0:
			g_ransac = new RansacFeatureSetMatcher(getParameter().acceptanceSigma * getParameter().acceptanceSigma * 5.99, 
				getParameter().success, getParameter().inlier,  getParameter().matchingThreshold, getParameter().acceptanceSigma * getParameter().acceptanceSigma * 3.84, false);
			break;
		case 1:
			g_ransac = new RansacMultiFeatureSetMatcher(getParameter().acceptanceSigma * getParameter().acceptanceSigma * 5.99, 
				getParameter().success, getParameter().inlier,  getParameter().matchingThreshold, getParameter().acceptanceSigma * getParameter().acceptanceSigma * 3.84, false);
			break;
		default:
			cerr<<"wrong match strategy!"<<endl;
			exit(-1);
	}
}
CFliterNode::_Parameters::_Parameters(){
	scale = 5; dmst = 2; window = 3; detectorType = 0; descriptorType = 0; distanceType = 2; matchStrategy = 0;
	 baseSigma = 0.2; sigmaStep = 1.4; minPeak = 0.34; minPeakDistance = 0.001; acceptanceSigma = 0.1; success = 0.95; inlier = 0.4; matchingThreshold = 0.4;
	 useMaxRange = false;
}

CFliterNode::FliterParameters& CFliterNode::getParameter(){
	static CFliterNode::FliterParameters g_singleParamter;
	return g_singleParamter;
}

PMScan* CFliterNode::fromLR2PM(LaserReading* lread){
	// std::cout<<"Oops, This function has not been defined!"<<std::endl;
	
	

	return NULL;
}

LaserReading* CFliterNode::fromPM2LR(PMScan* pmscan)
{
	if(pmscan==NULL){
		std::cout<<"pmscan is NULL"<<std::endl;
		return NULL;
	}
	if(pmscan->np != m_pFMatch->m_pParam->pm_l_points){
		std::cout<<"error: this scan is not compatible with Laser: "<<m_pFMatch->m_pParam->pm_laser_name<<std::endl;
		return NULL;
	}
	std::vector<double> rho(pmscan->np); // range value
	std::vector<double> phi(pmscan->np); // angle value
	std::vector<double> remission; 

	std::string sensorName(m_pFMatch->m_pParam->pm_laser_name);
	std::string robotName("");
	double timestamp = pmscan->t;
	for(int i=0;i<rho.size();i++)
	{
		rho[i] = pmscan->r[i]/100;
		phi[i] = m_pFMatch->pm_fi[i];
	}

	LaserReading * ret = new LaserReading(phi,rho,timestamp,sensorName,robotName);
	ret->setMaxRange(m_pFMatch->m_pParam->pm_max_range);
	ret->setLaserPose(OrientedPoint2D(pmscan->rx,pmscan->ry,pmscan->th));
	return ret;
}
