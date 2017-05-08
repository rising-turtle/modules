/*****************************************************************
 *
 * This file is part of the FLIRTLib project
 *
 * FLIRTLib Copyright (c) 2010 Gian Diego Tipaldi and Kai O. Arras 
 *
 * This software is licensed under the "Creative Commons 
 * License (Attribution-NonCommercial-ShareAlike 3.0)" 
 * and is copyrighted by Gian Diego Tipaldi and Kai O. Arras 
 * 
 * Further information on this license can be found at:
 * http://creativecommons.org/licenses/by-nc-sa/3.0/
 * 
 * FLIRTLib is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied 
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE.  
 *
 *****************************************************************/



#ifndef RANGEDETECTOR_H_
#define RANGEDETECTOR_H_

#include <feature/InterestPoint.h>
#include <feature/Detector.h>
#include <feature/MultiScaleDetector.h>
#include <utils/Convolution.h>
#include <utils/PeakFinder.h>

#include <vector>

/**
 * Representation of the range based detector.
 * The class represents the range based detector defined in the paper. It extracts blobs (extrema of the second derivative) on the range signal. 
 * This is equivalent of applying the theory behind the SIFT detector on the monodimensional range image.
 *
 * @author Gian Diego Tipaldi
 */

class RangeDetector: public MultiScaleDetector {
    public:
		/** 
		 * Constructor. Constructs and initialize the range based detector. 
		 *
		 * @param peak The peak finder used to detect maxima in the signal.
		 * @param scales The number of different scales to consider.
		 * @param sigma The standard deviation of the smoothing kernel for the initial scale (\f$ t_0 \f$ in the paper). 
		 * @param step The scale increment at every new scale (\f$ t_i \f$ in the paper). The standard deviation of the kernel at scale \f$ s \f$ is \f$ t_0 \cdot (t_i)^s \f$
		 * @param filterType The smoothing kernel used in the detector.
		 */
		RangeDetector(const PeakFinder* peak, unsigned int scales = 5, double sigma = 1.6, double step = 1.4, SmoothingFilterFamily filterType = BESSEL);

	/** Virtual Default destructor. */
	virtual ~RangeDetector() { }
	
    protected:
		/** Computes the bank for the second derivative at different scales. */
		virtual void computeDifferentialBank();
		
		virtual void computeSignal(const LaserReading& reading, std::vector<double>& signal, std::vector<unsigned int>& maxRangeMapping) const;
		
		virtual unsigned int computeInterestPoints(const LaserReading& reading, const std::vector<double>& signal, std::vector<InterestPoint*>& point, 
						   std::vector< std::vector<unsigned int> >& indexes, std::vector<unsigned int>& maxRangeMapping) const;
};

#endif
