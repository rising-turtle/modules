//============================================================================
// Name        : Helloworld.cpp
// Author      :
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================
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

#include <iostream>
#include <string>
#include <string.h>
#include <sstream>
#include <utility>

using namespace std;
namespace{
LogSensorStream m_sensorReference(NULL,NULL);

CurvatureDetector *m_detectorCurvature = NULL;
NormalBlobDetector *m_detectorNormalBlob = NULL;
NormalEdgeDetector *m_detectorNormalEdge = NULL;
RangeDetector *m_detectorRange = NULL;
Detector* m_detector = NULL;

BetaGridGenerator *m_betaGenerator = NULL;
ShapeContextGenerator *m_shapeGenerator = NULL;
DescriptorGenerator *m_descriptor = NULL;

RansacFeatureSetMatcher *m_ransac = NULL;

double angErrorTh = 0.2;
double linErrorTh = 0.5;

std::vector< std::vector<InterestPoint *> > m_pointsReference;
std::vector< OrientedPoint2D > m_posesReference;

unsigned int corresp[] = {0, 3, 5, 7, 9, 11, 13, 15};

double m_error[8] = {0.}, m_errorC[8] = {0.}, m_errorR[8] = {0.};
unsigned int m_match[8] = {0}, m_matchC[8] = {0}, m_matchR[8] = {0};
unsigned int m_valid[8] = {0};

//struct timeval detectTime, describeTime, ransacTime;

unsigned int m_localSkip = 1;


void match(unsigned int position)
{
	m_sensorReference.seek(position);

	std::vector<InterestPoint *> pointsLocal(m_pointsReference[position].size());
	const LaserReading* lreadReference = dynamic_cast<const LaserReading*>(m_sensorReference.current());
	for(unsigned int j = 0; j < m_pointsReference[position].size(); j++){
		InterestPoint * point = new InterestPoint(*m_pointsReference[position][j]);
		point->setPosition(lreadReference->getLaserPose().ominus(point->getPosition()));
		pointsLocal[j] = point;
	}

	/*unsigned int inliers[m_pointsReference.size()];
	double results[m_pointsReference.size()];
	double linearErrors[m_pointsReference.size()];
	double angularErrors[m_pointsReference.size()];*/

	std::vector<unsigned int> inliers(m_pointsReference.size());
	std::vector<double> results(m_pointsReference.size());
	std::vector<double> linearErrors(m_pointsReference.size());
	std::vector<double> angularErrors(m_pointsReference.size());

	//struct timeval start, end, diff, sum;
	for(unsigned int i = 0; i < m_pointsReference.size(); i++){
		if(fabs(double(i) - double(position)) < m_localSkip) {
			results[i] = 1e17;
			inliers[i] = 0;
			linearErrors[i] = 1e17;
			angularErrors[i] = 1e17;
			continue;
		}
		OrientedPoint2D transform;
		std::vector< std::pair<InterestPoint*, InterestPoint* > > correspondences;
	//	gettimeofday(&start,NULL);
		// 	std::cout << m_pointsReference[i].size() << " vs. " << pointsLocal.size() << std::endl;
		results[i] = m_ransac->matchSets(m_pointsReference[i], pointsLocal, transform, correspondences);
	/*	gettimeofday(&end,NULL);
		timersub(&end, &start, &diff);
		timeradd(&ransacTime, &diff, &sum);*/
		//ransacTime = sum;
		inliers[i] = correspondences.size();
		OrientedPoint2D delta = m_posesReference[position] - transform;
		linearErrors[i] = correspondences.size() ? delta * delta : 1e17;
		angularErrors[i] = correspondences.size() ? delta.theta * delta.theta : 1e17;
	}

	for(unsigned int c = 0; c < 8; c++){
		unsigned int maxCorres = 0;
		double maxResult = 1e17;
		double linError = 1e17, angError = 1e17;
		double linErrorC = 1e17, angErrorC = 1e17;
		double linErrorR = 1e17, angErrorR = 1e17;
		bool valid = false;
		for(unsigned int i = 0; i < m_pointsReference.size(); i++){
			if(linError + angError > linearErrors[i] + angularErrors[i]) {
				linError = linearErrors[i];
				angError = angularErrors[i];
			}
			if(maxCorres < inliers[i]){
				linErrorC = linearErrors[i];
				angErrorC = angularErrors[i];
				maxCorres = inliers[i];
			}
			if(maxResult > results[i]){
				linErrorR = linearErrors[i];
				angErrorR = angularErrors[i];
				maxResult = results[i];
			}
			valid = valid || inliers[i] >= corresp[c];
		}


		if(valid){
			m_match[c] += (linError <= (linErrorTh * linErrorTh) && angError <= (angErrorTh * angErrorTh) );
			m_matchC[c] += (linErrorC <= (linErrorTh * linErrorTh) && angErrorC <= (angErrorTh * angErrorTh) );
			m_matchR[c] += (linErrorR <= (linErrorTh * linErrorTh) && angErrorR <= (angErrorTh * angErrorTh) );

			m_error[c] += sqrt(linError + angError);
			m_errorC[c] += sqrt(linErrorC + angErrorC);
			m_errorR[c] += sqrt(linErrorR + angErrorR);

			m_valid[c]++;
		}
	}

}

void processOneByOne(){
	unsigned int i = 0;
	unsigned int position = m_sensorReference.tell();
	m_sensorReference.seek(0,END);
	unsigned int last = m_sensorReference.tell();
	m_sensorReference.seek(0);

	std::string bar(50, ' ');
	bar[0] = '#';
	unsigned int progress = 0;

	//unsigned int inliers[m_pointsReference.size()];
	//double results[m_pointsReference.size()];
	//double linearErrors[m_pointsReference.size()];
	//double angularErrors[m_pointsReference.size()];
	//struct timeval start, end, diff, sum;

	std::vector<unsigned int> inliers(m_pointsReference.size());
	std::vector<double> results(m_pointsReference.size());
	std::vector<double> linearErrors(m_pointsReference.size());
	std::vector<double> angularErrors(m_pointsReference.size());

	std::ofstream matchOut("pose.txt");
	std::ofstream errorOut("error.txt");
	std::ofstream timeOut("time.txt");

	while(!m_sensorReference.end()){
		cout<<"start:	"<<m_sensorReference.tell()<<endl;
		unsigned int currentProgress = (m_sensorReference.tell()*100)/last;
		if (progress < currentProgress){
			progress = currentProgress;
			bar[progress/2] = '#';
			std::cout << "\rDetecting points  [" << bar << "] " << (m_sensorReference.tell()*100)/last << "%" << std::endl;
		}

		const LaserReading* lreadReference = dynamic_cast<const LaserReading*>(m_sensorReference.next());
		if (lreadReference){
			//Detect
			//gettimeofday(&start, NULL);
			m_detector->detect(*lreadReference, m_pointsReference[i]);
			m_posesReference[i] = lreadReference->getLaserPose();
			/*gettimeofday(&end,NULL);
			timersub(&end,&start,&detectTime);*/
			//timeOut<<double(detectTime.tv_sec) + 1e-06 * double(detectTime.tv_usec)<<"\t";
			cout<<"number of detected points: "<<m_pointsReference[i].size()<<endl;

			//Descriptor
			//gettimeofday(&start, NULL);
			for(unsigned int j = 0; j < m_pointsReference[i].size(); j++){
				m_pointsReference[i][j]->setDescriptor(m_descriptor->describe(*m_pointsReference[i][j], *lreadReference));
			}
			std::vector<InterestPoint *> pointsLocal(m_pointsReference[i].size());
			for(unsigned int j = 0; j < m_pointsReference[i].size(); j++){
				InterestPoint * point = new InterestPoint(*m_pointsReference[i][j]);
				point->setPosition(lreadReference->getLaserPose().ominus(point->getPosition()));
				pointsLocal[j] = point;
			}
		
			/*gettimeofday(&end,NULL);
			timersub(&end,&start,&detectTime);
			timeOut<<double(detectTime.tv_sec) + 1e-06 * double(detectTime.tv_usec)<<"\t";*/

			//matching
			if(i>0){

				cout<<"start ransac"<<endl;
				OrientedPoint2D transform;
				std::vector< std::pair<InterestPoint*, InterestPoint* > > correspondences;
				// 	std::cout << m_pointsReference[i-1].size() << " vs. " << m_pointsReference[i].size() << std::endl;
				//gettimeofday(&start, NULL);
				results[i] = m_ransac->matchSets(m_pointsReference[i-1], pointsLocal, transform, correspondences);
				cout<<results[i]<<endl;
				//gettimeofday(&end,NULL);
				//timersub(&end,&start,&detectTime);
				//timeOut<<double(detectTime.tv_sec) + 1e-06 * double(detectTime.tv_usec)<<std::endl;
				cout<<"end ransac"<<endl;

				inliers[i] = correspondences.size();
				OrientedPoint2D delta = m_posesReference[i] - transform;
				linearErrors[i] = correspondences.size() ? delta * delta : 1e17;
				angularErrors[i] = correspondences.size() ? delta.theta * delta.theta : 1e17;

				matchOut<<transform.x<<"\t"<<transform.y<<"\t"<<transform.theta<<endl;
				errorOut<<inliers[i]<<"\t"<<linearErrors[i]<<"\t"<<angularErrors[i]<<endl;
			}
		}
		else
		{
			cout<<"no data processing"<<endl;
		}
		i++;
		cout<<"end:	"<<m_sensorReference.tell()<<endl;
	}
	//gettimeofday(&end,NULL);
	//timersub(&end,&start,&detectTime);

	std::cout << " done." << std::endl;

}


void detectLog(){
	unsigned int i = 0;
	unsigned int position = m_sensorReference.tell();
	m_sensorReference.seek(0,END);
	unsigned int last = m_sensorReference.tell();
	m_sensorReference.seek(0);

	std::string bar(50, ' ');
	bar[0] = '#';
	unsigned int progress = 0;

	//struct timeval start, end;
	//gettimeofday(&start, NULL);
	while(!m_sensorReference.end()){
		unsigned int currentProgress = (m_sensorReference.tell()*100)/last;
		if (progress < currentProgress){
			progress = currentProgress;
			bar[progress/2] = '#';
			std::cout << "\rDetecting points  [" << bar << "] " << (m_sensorReference.tell()*100)/last << "%" << std::flush;
		}
		const LaserReading* lreadReference = dynamic_cast<const LaserReading*>(m_sensorReference.next());
		if (lreadReference){
			m_detector->detect(*lreadReference, m_pointsReference[i]);
			m_posesReference[i] = lreadReference->getLaserPose();
			i++;
		}
	}
	//gettimeofday(&end,NULL);
	//timersub(&end,&start,&detectTime);
	m_sensorReference.seek(position);
	std::cout << " done." << std::endl;
}

void countLog(){
	double flirtNum = 0.;
	uint count = 0;

	std::string bar(50, ' ');
	bar[0] = '#';

	unsigned int progress = 0;
	for(unsigned int i = 0; i < m_pointsReference.size(); i++){
		unsigned int currentProgress = (i*100)/(m_pointsReference.size() - 1);
		if (progress < currentProgress){
			progress = currentProgress;
			bar[progress/2] = '#';
			std::cout << "\rCounting points  [" << bar << "] " << progress << "%" << std::flush;
		}
		if(m_pointsReference[i].size()){
			flirtNum += m_pointsReference[i].size();
			count++;
		}
	}
	flirtNum=count?flirtNum/double(count):0.;
	std::cout << " done.\nFound " << flirtNum << " FLIRT features per scan." << std::endl;
}

void describeLog(){
	unsigned int i = 0;
	unsigned int position = m_sensorReference.tell();
	m_sensorReference.seek(0,END);
	unsigned int last = m_sensorReference.tell();
	m_sensorReference.seek(0);

	std::string bar(50, ' ');
	bar[0] = '#';
	//struct timeval start, end;
	//gettimeofday(&start, NULL);
	unsigned int progress = 0;

	while(!m_sensorReference.end()){
		unsigned int currentProgress = (m_sensorReference.tell()*100)/last;
		if (progress < currentProgress){
			progress = currentProgress;
			bar[progress/2] = '#';
			std::cout << "\rDescribing points  [" << bar << "] " << progress << "%" << std::flush;
		}
		const LaserReading* lreadReference = dynamic_cast<const LaserReading*>(m_sensorReference.next());
		if (lreadReference){
			for(unsigned int j = 0; j < m_pointsReference[i].size(); j++){
				m_pointsReference[i][j]->setDescriptor(m_descriptor->describe(*m_pointsReference[i][j], *lreadReference));
			}
			i++;
		}
	}
	/*gettimeofday(&end,NULL);
	timersub(&end,&start,&describeTime);*/
	m_sensorReference.seek(position);
	std::cout << " done." << std::endl;
}
};

int testFlirt() {
	//std::string filename("/home/liu/Projects/openslam/flirtlib/trunk/data/intel-lab_small.log");
	std::string filename("D:\\exprdata\\slamlog\\intel-lab.log");
	unsigned int scale = 5, dmst = 2;//, window = 3;
	double baseSigma = 0.2, sigmaStep = 1.4, minPeak = 0.34, minPeakDistance = 0.001, acceptanceSigma = 0.1, success = 0.95, inlier = 0.4, matchingThreshold = 0.4;
	bool useMaxRange = false;

	CarmenLogWriter writer;
	CarmenLogReader reader;

	m_sensorReference = LogSensorStream(&reader, &writer);
	m_sensorReference.load(filename);

	SimpleMinMaxPeakFinder *m_peakMinMax = new SimpleMinMaxPeakFinder(minPeak, minPeakDistance);

	std::string detector("curvature");
	m_detectorCurvature = new CurvatureDetector(m_peakMinMax, scale, baseSigma, sigmaStep, dmst);
	m_detectorCurvature->setUseMaxRange(useMaxRange);
	m_detector = m_detectorCurvature;

	HistogramDistance<double> *dist = NULL;
	std::string distance("symchi2");
	dist = new SymmetricChi2Distance<double>();

	std::string descriptor("beta");
	m_betaGenerator = new BetaGridGenerator(0.02, 0.5, 4, 12);
	m_betaGenerator->setDistanceFunction(dist);
	m_descriptor = m_betaGenerator;

	m_ransac = new RansacFeatureSetMatcher(acceptanceSigma * acceptanceSigma * 5.99, success, inlier, matchingThreshold, acceptanceSigma * acceptanceSigma * 3.84, false);
	//m_ransac = new RansacMultiFeatureSetMatcher(acceptanceSigma * acceptanceSigma * 5.99, success, inlier, matchingThreshold, acceptanceSigma * acceptanceSigma * 3.84, false);


	std::cerr << "Processing file:\t" << filename << "\nDetector:\t\t" << detector << "\nDescriptor:\t\t" << descriptor << "\nDistance:\t\t" << distance << std::endl;

	m_sensorReference.seek(0,END);
	unsigned int end = m_sensorReference.tell();
	m_sensorReference.seek(0,BEGIN);

	m_pointsReference.resize(end + 1);
	m_posesReference.resize(end + 1);

	processOneByOne();

	return 0;
}


/*
int main(int argc, char **argv){

	std::string filename("/home/liu/Projects/openslam/flirtlib/trunk/data/intel-lab_small.log");
	unsigned int scale = 5, dmst = 2;//, window = 3;
	double baseSigma = 0.2, sigmaStep = 1.4, minPeak = 0.34, minPeakDistance = 0.001, acceptanceSigma = 0.1, success = 0.95, inlier = 0.4, matchingThreshold = 0.4;
	bool useMaxRange = false;

	CarmenLogWriter writer;
	CarmenLogReader reader;

	m_sensorReference = LogSensorStream(&reader, &writer);
	m_sensorReference.load(filename);

	SimpleMinMaxPeakFinder *m_peakMinMax = new SimpleMinMaxPeakFinder(minPeak, minPeakDistance);

	std::string detector("curvature");
	m_detectorCurvature = new CurvatureDetector(m_peakMinMax, scale, baseSigma, sigmaStep, dmst);
	m_detectorCurvature->setUseMaxRange(useMaxRange);
	m_detector = m_detectorCurvature;

	HistogramDistance<double> *dist = NULL;
	std::string distance("symchi2");
	dist = new SymmetricChi2Distance<double>();

	std::string descriptor("beta");
	m_betaGenerator = new BetaGridGenerator(0.02, 0.5, 4, 12);
	m_betaGenerator->setDistanceFunction(dist);
	m_descriptor = m_betaGenerator;

	m_ransac = new RansacFeatureSetMatcher(acceptanceSigma * acceptanceSigma * 5.99, success, inlier, matchingThreshold, acceptanceSigma * acceptanceSigma * 3.84, false);
	//m_ransac = new RansacMultiFeatureSetMatcher(acceptanceSigma * acceptanceSigma * 5.99, success, inlier, matchingThreshold, acceptanceSigma * acceptanceSigma * 3.84, false);


	std::cerr << "Processing file:\t" << filename << "\nDetector:\t\t" << detector << "\nDescriptor:\t\t" << descriptor << "\nDistance:\t\t" << distance << std::endl;

	m_sensorReference.seek(0,END);
	unsigned int end = m_sensorReference.tell();
	m_sensorReference.seek(0,BEGIN);

	m_pointsReference.resize(end + 1);
	m_posesReference.resize(end + 1);

	detectLog();

	countLog();

	describeLog();

	std::string outfile = filename;

	timerclear(&ransacTime);

	std::string bar(50,' ');
	bar[0] = '#';
	unsigned int progress = 0;

	for(unsigned int i =0; i < m_pointsReference.size(); i++){
		unsigned int currentProgress = (i*100)/(m_pointsReference.size() - 1);
		if (progress < currentProgress){
			progress = currentProgress;
			bar[progress/2] = '#';
			std::cout << "\rMatching  [" << bar << "] " << progress << "%" << std::flush;
		}
		match(i);
	}
	std::cout << " done." << std::endl;

	std::stringstream matchFile;
	std::stringstream errorFile;
	std::stringstream timeFile;
	matchFile << outfile << "_" << detector << "_" << descriptor << "_" << distance << "_match.dat";
	errorFile << outfile << "_" << detector << "_" << descriptor << "_" << distance << "_error.dat";
	timeFile << outfile << "_" << detector << "_" << descriptor << "_" << distance << "_time.dat";

	std::ofstream matchOut(matchFile.str().c_str());
	std::ofstream errorOut(errorFile.str().c_str());
	std::ofstream timeOut(timeFile.str().c_str());

	matchOut << "# Number of matches according to various strategies" << std::endl;
	matchOut << "# The valid matches are the one with at least n correspondences in the inlier set " << std::endl;
	matchOut << "# where n = {0, 3, 5, 7, 9, 11, 13, 15}, one for each line " << std::endl;
	matchOut << "# optimal \t correspondence \t residual \v valid" << std::endl;

	errorOut << "# Mean error according to various strategies" << std::endl;
	errorOut << "# The valid matches are the one with at least n correspondences in the inlier set " << std::endl;
	errorOut << "# where n = {0, 3, 5, 7, 9, 11, 13, 15}, one for each line " << std::endl;
	errorOut << "# optimal \t correspondence \t residual \v valid" << std::endl;

	timeOut << "# Total time spent for the various steps" << std::endl;
	timeOut << "# detection \t description \t RANSAC" << std::endl;

	for(unsigned int c = 0; c < 8; c++){
		matchOut << m_match[c] << "\t" << m_matchC[c] << "\t" << m_matchR[c] << "\t" << m_valid[c] << std::endl;
		errorOut << m_error[c]/m_valid[c] << "\t" << m_errorC[c]/m_valid[c] << "\t" << m_errorR[c]/m_valid[c] << "\t" << m_valid[c] << std::endl;
	}
	timeOut << double(detectTime.tv_sec) + 1e-06 * double(detectTime.tv_usec) << "\t"
			<< double(describeTime.tv_sec) + 1e-06 * double(describeTime.tv_usec) << "\t"
			<< double(ransacTime.tv_sec) + 1e-06 * double(ransacTime.tv_usec) << std::endl;
}
*/

