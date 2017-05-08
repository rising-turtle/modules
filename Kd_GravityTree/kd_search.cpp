/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                   http://mrpt.sourceforge.net/                            |
   |                                                                           |
   |   Copyright (C) 2005-2010  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
   +---------------------------------------------------------------------------+ */


//----------------------------------------------------------------------
// File:			kd_search.cpp
// Programmer:		Sunil Arya and David Mount
// Description:		Standard kd-tree search
// Last modified:	01/04/05 (Version 1.0)
//----------------------------------------------------------------------
// Copyright (c) 1997-2005 University of Maryland and Sunil Arya and
// David Mount.  All Rights Reserved.
//
// This software and related documentation is part of the Approximate
// Nearest Neighbor Library (ANN).  This software is provided under
// the provisions of the Lesser GNU Public License (LGPL).  See the
// file ../ReadMe.txt for further information.
//
// The University of Maryland (U.M.) and the authors make no
// representations about the suitability or fitness of this software for
// any purpose.  It is provided "as is" without express or implied
// warranty.
//----------------------------------------------------------------------
// History:
//	Revision 0.1  03/04/98
//		Initial release
//	Revision 1.0  04/01/05
//		Changed names LO, HI to ANN_LO, ANN_HI
//----------------------------------------------------------------------

#include "ann/kd_search.h"					// kd-search declarations

//----------------------------------------------------------------------
//	Approximate nearest neighbor searching by kd-tree search
//		The kd-tree is searched for an approximate nearest neighbor.
//		The point is returned through one of the arguments, and the
//		distance returned is the squared distance to this point.
//
//		The method used for searching the kd-tree is an approximate
//		adaptation of the search algorithm described by Friedman,
//		Bentley, and Finkel, ``An algorithm for finding best matches
//		in logarithmic expected time,'' ACM Transactions on Mathematical
//		Software, 3(3):209-226, 1977).
//
//		The algorithm operates recursively.  When first encountering a
//		node of the kd-tree we first visit the child which is closest to
//		the query point.  On return, we decide whether we want to visit
//		the other child.  If the box containing the other child exceeds
//		1/(1+eps) times the current best distance, then we skip it (since
//		any point found in this child cannot be closer to the query point
//		by more than this factor.)  Otherwise, we visit it recursively.
//		The distance between a box and the query point is computed exactly
//		(not approximated as is often done in kd-tree), using incremental
//		distance updates, as described by Arya and Mount in ``Algorithms
//		for fast vector quantization,'' Proc.  of DCC '93: Data Compression
//		Conference, eds. J. A. Storer and M. Cohn, IEEE Press, 1993,
//		381-390.
//
//		The main entry points is annkSearch() which sets things up and
//		then call the recursive routine ann_search().  This is a recursive
//		routine which performs the processing for one node in the kd-tree.
//		There are two versions of this virtual procedure, one for splitting
//		nodes and one for leaves.  When a splitting node is visited, we
//		determine which child to visit first (the closer one), and visit
//		the other child on return.  When a leaf is visited, we compute
//		the distances to the points in the buckets, and update information
//		on the closest points.
//
//		Some trickery is used to incrementally update the distance from
//		a kd-tree rectangle to the query point.  This comes about from
//		the fact that which each successive split, only one component
//		(along the dimension that is split) of the squared distance to
//		the child rectangle is different from the squared distance to
//		the parent rectangle.
//----------------------------------------------------------------------

//----------------------------------------------------------------------
//		To keep argument lists short, a number of global variables
//		are maintained which are common to all the recursive calls.
//		These are given below.
//----------------------------------------------------------------------

int				ANNkdDim;				// dimension of space
ANNpoint		ANNkdQ;					// query point
double			ANNkdMaxErr;			// max tolerable squared error
ANNpointArray	ANNkdPts;				// the points
ANNmin_k		*ANNkdPointMK;			// set of k closest points

//----------------------------------------------------------------------
//	annkSearch - search for the k nearest neighbors
//----------------------------------------------------------------------

void ANNkd_tree::annkSearch(
	ANNpoint			q,				// the query point
	int					k,				// number of near neighbors to return
	ANNidxArray			nn_idx,			// nearest neighbor indices (returned)
	ANNdistArray		dd,				// the approximate nearest neighbor
	double				eps)			// the error bound
{

	ANNkdDim = dim;						// copy arguments to static equivs
	ANNkdQ = q;
	ANNkdPts = pts;
	ANNptsVisited = 0;					// initialize count of points visited

	if (k > n_pts) {					// too many near neighbors?
		annError("Requesting more near neighbors than data points", ANNabort);
	}

	ANNkdMaxErr = ANN_POW(1.0 + eps);
	ANN_FLOP(2)							// increment floating op count

	ANNkdPointMK = new ANNmin_k(k);		// create set for closest k points
										// search starting at the root
	root->ann_search(annBoxDistance(q, bnd_box_lo, bnd_box_hi, dim));

	for (int i = 0; i < k; i++) {		// extract the k-th closest points
		dd[i] = ANNkdPointMK->ith_smallest_key(i);
		nn_idx[i] = ANNkdPointMK->ith_smallest_info(i);
	}
	delete ANNkdPointMK;				// deallocate closest point set
}

//----------------------------------------------------------------------
//	annkSearch1 - search for the nearest gravity_point
//----------------------------------------------------------------------
void ANNkd_tree::annkSearch1(
	ANNpoint			q,				// the query point
	float &out_x,float &out_y,float &out_z,			// nearest gravity_point coordinates(returned)
	ANNdist         	&dd,				// the approximate nearest neighbor
	double				eps)			// the error bound
{

	ANNkdDim = dim;						// copy arguments to static equivs
	ANNkdQ = q;
	ANNkdPts = pts;
	ANNptsVisited = 0;					// initialize count of points visited

	ANNkdMaxErr = ANN_POW(1.0 + eps);
	ANN_FLOP(2)							// increment floating op count

	ANNkdPointMK = new ANNmin_k(1);		// create set for closest k points
										// search starting at the root
	root->ann_search1(annBoxDistance(q, bnd_box_lo, bnd_box_hi, dim));

	// extract the closest gravity_point
	dd = ANNkdPointMK->ith_smallest_key(0); 
	out_x = ANNkdPointMK->g_node->gravity_p[0];
	out_y = ANNkdPointMK->g_node->gravity_p[1];
	out_z = ANNkdPointMK->g_node->gravity_p[2];

	delete ANNkdPointMK;				// deallocate closest point set
}
//----------------------------------------------------------------------
//	kd_split::ann_search - search a splitting node
//----------------------------------------------------------------------

void ANNkd_split::ann_search(ANNdist box_dist)
{
										// check dist calc term condition
	if (ANNmaxPtsVisited != 0 && ANNptsVisited > ANNmaxPtsVisited) return;

										// distance to cutting plane
	ANNcoord cut_diff = ANNkdQ[cut_dim] - cut_val;

	if (cut_diff < 0) {					// left of cutting plane
		child[ANN_LO]->ann_search(box_dist);// visit closer child first

		ANNcoord box_diff = cd_bnds[ANN_LO] - ANNkdQ[cut_dim];
		if (box_diff < 0)				// within bounds - ignore
			box_diff = 0;
										// distance to further box
		box_dist = (ANNdist) ANN_SUM(box_dist,
				ANN_DIFF(ANN_POW(box_diff), ANN_POW(cut_diff)));

										// visit further child if close enough
		if (box_dist * ANNkdMaxErr < ANNkdPointMK->max_key())
			child[ANN_HI]->ann_search(box_dist);

	}
	else {								// right of cutting plane
		child[ANN_HI]->ann_search(box_dist);// visit closer child first

		ANNcoord box_diff = ANNkdQ[cut_dim] - cd_bnds[ANN_HI];
		if (box_diff < 0)				// within bounds - ignore
			box_diff = 0;
										// distance to further box
		box_dist = (ANNdist) ANN_SUM(box_dist,
				ANN_DIFF(ANN_POW(box_diff), ANN_POW(cut_diff)));

										// visit further child if close enough
		if (box_dist * ANNkdMaxErr < ANNkdPointMK->max_key())
			child[ANN_LO]->ann_search(box_dist);

	}
	ANN_FLOP(10)						// increment floating ops
	ANN_SPL(1)							// one more splitting node visited
}
void ANNkd_split::ann_search1(ANNdist box_dist)
{
										// check dist calc term condition
	if (ANNmaxPtsVisited != 0 && ANNptsVisited > ANNmaxPtsVisited) return;

										// distance to cutting plane
	ANNcoord cut_diff = ANNkdQ[cut_dim] - cut_val;

	if (cut_diff < 0) {					// left of cutting plane
		child[ANN_LO]->ann_search1(box_dist);// visit closer child first

		ANNcoord box_diff = cd_bnds[ANN_LO] - ANNkdQ[cut_dim];
		if (box_diff < 0)				// within bounds - ignore
			box_diff = 0;
										// distance to further box
		box_dist = (ANNdist) ANN_SUM(box_dist,
				ANN_DIFF(ANN_POW(box_diff), ANN_POW(cut_diff)));

										// visit further child if close enough
		if (box_dist * ANNkdMaxErr < ANNkdPointMK->max_key())
			child[ANN_HI]->ann_search1(box_dist);

	}
	else {								// right of cutting plane
		child[ANN_HI]->ann_search1(box_dist);// visit closer child first

		ANNcoord box_diff = ANNkdQ[cut_dim] - cd_bnds[ANN_HI];
		if (box_diff < 0)				// within bounds - ignore
			box_diff = 0;
										// distance to further box
		box_dist = (ANNdist) ANN_SUM(box_dist,
				ANN_DIFF(ANN_POW(box_diff), ANN_POW(cut_diff)));

										// visit further child if close enough
		if (box_dist * ANNkdMaxErr < ANNkdPointMK->max_key())
			child[ANN_LO]->ann_search1(box_dist);

	}
	ANN_FLOP(10)						// increment floating ops
	ANN_SPL(1)							// one more splitting node visited
}
//----------------------------------------------------------------------
//	kd_leaf::ann_search - search points in a leaf node
//		Note: The unreadability of this code is the result of
//		some fine tuning to replace indexing by pointer operations.
//----------------------------------------------------------------------

void ANNkd_leaf::ann_search1(ANNdist box_dist)
{
	register ANNdist dist;				// distance to data point
	register ANNcoord* pp;				// data coordinate pointer
	register ANNcoord* qq;				// query coordinate pointer
	register ANNdist min_dist;			// distance to k-th closest point
	register ANNcoord t;
	register int d;

	min_dist = ANNkdPointMK->max_key(); // k-th smallest distance so far

	dist = 0;
	if(!gravity_p) // if The gravity point has not been calculated
							// then calculate it
	{
		gravity_p = new ANNcoord[ANNkdDim];
		for(d = 0; d< ANNkdDim; d++) gravity_p[d] = 0.0;
			for(int i = 0; i < n_pts; i++)
			{
				pp = ANNkdPts[bkt[i]];			// first coord of next data point
				for( d = 0; d<ANNkdDim; d++ )
				{
						gravity_p[d] += pp[d];
				}
			}
			if(n_pts > 1){
				for(d = 0; d< ANNkdDim; d++) {
					gravity_p[d] /= n_pts;
				}
				}
	}
	    qq = ANNkdQ;					// first coord of query point
		for(d = 0; d < ANNkdDim; d++) {
					ANN_COORD(1)				// one more coordinate hit
					ANN_FLOP(4)					// increment floating ops

					//t = *(qq++) - *(gravity_p++);		// compute length and adv coordinate
					t = qq[d] - gravity_p[d];
										// exceeds dist to k-th smallest?
					if( (dist = ANN_SUM(dist, ANN_POW(t))) > min_dist) {
						break;
					}
			}
			if (d >= ANNkdDim &&					// among the k best?
		   (ANN_ALLOW_SELF_MATCH || dist!=0)) { // and no self-match problem
												// add it to the list
			ANNkdPointMK->insert(dist, gravity_p);// here using the first point of the bucket to record gravity_point
			min_dist = ANNkdPointMK->max_key();
		}
		
	ANN_LEAF(1)							// one more leaf node visited
	ANN_PTS(n_pts)						// increment points visited
	ANNptsVisited += n_pts;				// increment number of points visited
}
void ANNkd_leaf::ann_search(ANNdist box_dist)
{
	register ANNdist dist;				// distance to data point
	register ANNcoord* pp;				// data coordinate pointer
	register ANNcoord* qq;				// query coordinate pointer
	register ANNdist min_dist;			// distance to k-th closest point
	register ANNcoord t;
	register int d;

	min_dist = ANNkdPointMK->max_key(); // k-th smallest distance so far

	for (int i = 0; i < n_pts; i++) {	// check points in bucket

		pp = ANNkdPts[bkt[i]];			// first coord of next data point
		qq = ANNkdQ;					// first coord of query point
		dist = 0;

		for(d = 0; d < ANNkdDim; d++) {
			ANN_COORD(1)				// one more coordinate hit
			ANN_FLOP(4)					// increment floating ops

			t = *(qq++) - *(pp++);		// compute length and adv coordinate
										// exceeds dist to k-th smallest?
			if( (dist = ANN_SUM(dist, ANN_POW(t))) > min_dist) {
				break;
			}
		}

		if (d >= ANNkdDim &&					// among the k best?
		   (ANN_ALLOW_SELF_MATCH || dist!=0)) { // and no self-match problem
												// add it to the list
			ANNkdPointMK->insert(dist, bkt[i]);
			min_dist = ANNkdPointMK->max_key();
		}
	}
	ANN_LEAF(1)							// one more leaf node visited
	ANN_PTS(n_pts)						// increment points visited
	ANNptsVisited += n_pts;				// increment number of points visited
}

	//min_dist = ANNkdPointMK->max_key(); // k-th smallest distance so far

	//for (int i = 0; i < n_pts; i++) {	// check points in bucket

	//	pp = ANNkdPts[bkt[i]];			// first coord of next data point
	//	qq = ANNkdQ;					// first coord of query point
	//	dist = 0;

	//	for(d = 0; d < ANNkdDim; d++) {
	//		ANN_COORD(1)				// one more coordinate hit
	//		ANN_FLOP(4)					// increment floating ops

	//		t = *(qq++) - *(pp++);		// compute length and adv coordinate
	//									// exceeds dist to k-th smallest?
	//		if( (dist = ANN_SUM(dist, ANN_POW(t))) > min_dist) {
	//			break;
	//		}
	//	}

	//	if (d >= ANNkdDim &&					// among the k best?
	//	   (ANN_ALLOW_SELF_MATCH || dist!=0)) { // and no self-match problem
	//											// add it to the list
	//		ANNkdPointMK->insert(dist, bkt[i]);
	//		min_dist = ANNkdPointMK->max_key();
	//	}
	//}
	//ANN_LEAF(1)							// one more leaf node visited
	//ANN_PTS(n_pts)						// increment points visited
	//ANNptsVisited += n_pts;				// increment number of points visited