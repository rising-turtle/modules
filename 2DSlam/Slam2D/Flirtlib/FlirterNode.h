#ifndef FLIRTER_NODE_H
#define FLIRTER_NODE_H

#include "preheader.h"
#include <utils/HistogramDistances.h>
#include <geometry/point.h>

#include "ZHPolar_Match.h"
#define LASER_NAME "LMS151"

class InterestPoint;
class LaserReading;

// detectors 
class CurvatureDetector;
class NormalBlobDetector;
class NormalEdgeDetector;
class RangeDetector;
class Detector;

// distance
template<class N>
class HistogramDistance;
template<class N>
class EuclideanDistance;
template<class N>
class Chi2Distance;
template<class N>
class SymmetricChi2Distance;
template<class N>
class BatthacharyyaDistance;
template<class N>
class KullbackLeiblerDistance; 
template <class N>
class JensenShannonDistance;

// peakFinders
class SimpleMinMaxPeakFinder;


// generators 
class BetaGridGenerator;
class ShapeContextGenerator;
class DescriptorGenerator;

// feature matchers
class RansacFeatureSetMatcher;

class CFliterNode{
public:
	CFliterNode(LaserReading* lread, string laser_name=LASER_NAME);
	CFliterNode(PMScan* pmscan, string laser_name=LASER_NAME);
	~CFliterNode();
	void InitFliter();
	// detect peak points
	// describe peak points 
	// match with previous nodes

	void InitNode(LaserReading* lread); 
	void UnitNode();
	
	// identify whether all the parameters have been initialized
	static bool g_IsInit;
	static void resetInit();

	// peakFinders
	static SimpleMinMaxPeakFinder* g_peakMinMax;

	// detectors
	static string g_detector_name;
	static CurvatureDetector *g_detectorCurvature;	
	static NormalBlobDetector *g_detectorNormalBlob;
	static NormalEdgeDetector *g_detectorNormalEdge;
	static RangeDetector *g_detectorRange;
	static Detector *g_detector; 

	// distance
	static string g_distance_name;
	static HistogramDistance<double> *g_dist ;

	// descriptors
	static string g_descriptor_name;
	static BetaGridGenerator *g_betaGenerator;
	static ShapeContextGenerator *g_shapeGenerator;
	static DescriptorGenerator *g_descriptor ;
	
	// feature matcher
	static RansacFeatureSetMatcher *g_ransac;

	// parameters for Flirter Node
	typedef struct _Parameters{
		unsigned int scale;
		unsigned int dmst;
		unsigned int window;
		unsigned int detectorType;		// detector type	
		unsigned int descriptorType;		// descriptor type 
		unsigned int distanceType;			// distance type
		unsigned int matchStrategy;		// feature match strategy
		double baseSigma;
		double sigmaStep;
		double minPeak;
		double minPeakDistance;
		double acceptanceSigma; 
		double success;
		double inlier;
		double matchingThreshold;
		bool useMaxRange;
		struct _Parameters();
	}FliterParameters;

	static FliterParameters& getParameter();
public:
	  std::vector<InterestPoint *> m_featurePoints;	// feature points
	  std::vector<InterestPoint*> m_featurePointsLocal; // feature points in local reference
	  OrientedPoint2D	m_pose;					// pose
	  OrientedPoint2D m_relpose;					// relative pose to previous nodes
	  LaserReading * m_pLaserRead;				// laser reading info
public:	// add PSM match
	  CPolarMatch* m_pFMatch;	// frontend PSM or ICP Matching
	  PMScan* m_pScan;
public:
	bool matchNodePairLocal( CFliterNode* refNode, OrientedPoint2D& transform);
	bool matchNodePairGlobal( CFliterNode* refNode, OrientedPoint2D& transform);
	int matchFeaturePoints(std::vector<InterestPoint*>& fpref,std::vector<InterestPoint*>& fpcur,
		OrientedPoint2D& transform);
	bool matchNodeFrontend(CFliterNode* pref); // match with previous Node using PSM + ICP
private:
	// transfrom between lread and pmscan
	LaserReading* fromPM2LR(PMScan* pmscan);
	PMScan* fromLR2PM(LaserReading* lread);
};



#endif