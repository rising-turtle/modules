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
// File:			bd_search.cpp
// Programmer:		David Mount
// Description:		Standard bd-tree search
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
//----------------------------------------------------------------------

#include "ann/bd_tree.h"					// bd-tree declarations
#include "ann/kd_search.h"					// kd-tree search declarations

//----------------------------------------------------------------------
//	Approximate searching for bd-trees.
//		See the file kd_search.cpp for general information on the
//		approximate nearest neighbor search algorithm.  Here we
//		include the extensions for shrinking nodes.
//----------------------------------------------------------------------

//----------------------------------------------------------------------
//	bd_shrink::ann_search - search a shrinking node
//----------------------------------------------------------------------

void ANNbd_shrink::ann_search(ANNdist box_dist)
{
												// check dist calc term cond.
	if (ANNmaxPtsVisited != 0 && ANNptsVisited > ANNmaxPtsVisited) return;

	ANNdist inner_dist = 0;						// distance to inner box
	for (int i = 0; i < n_bnds; i++) {			// is query point in the box?
		if (bnds[i].out(ANNkdQ)) {				// outside this bounding side?
												// add to inner distance
			inner_dist = (ANNdist) ANN_SUM(inner_dist, bnds[i].dist(ANNkdQ));
		}
	}
	if (inner_dist <= box_dist) {				// if inner box is closer
		child[ANN_IN]->ann_search(inner_dist);	// search inner child first
		child[ANN_OUT]->ann_search(box_dist);	// ...then outer child
	}
	else {										// if outer box is closer
		child[ANN_OUT]->ann_search(box_dist);	// search outer child first
		child[ANN_IN]->ann_search(inner_dist);	// ...then outer child
	}
	ANN_FLOP(3*n_bnds)							// increment floating ops
	ANN_SHR(1)									// one more shrinking node
}
