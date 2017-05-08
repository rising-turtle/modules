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
#ifndef  KDTreeCapable_H
#define  KDTreeCapable_H

#include "preheader.h"

#include "./ann/ANN.h"  // ANN: for kd-tree

		/** A base virtual class providing automatic, cached KD-tree-based look-up of points among data of arbitrary dimensionality.
		 *  Any derived class must only implement:
		 * 		- kdtree_get_point_count()
		 * 		- kdtree_fill_point_data()
		 *  and must be aware of the need to call "kdtree_mark_as_outdated()" when the data points change to mark the cached KD-tree as invalid.
		 *
		 * The KD-tree will be built on demand only upon call of any of the query methods provided by
		 *  this class (kdTreeClosestPoint2D, etc.).
		 *
		 *  Notice that there is only ONE internal cached KD-tree, so if a method to query a 2D point is called,
		 *  then another method for 3D points, then again the 2D method, three KD-trees will be built. So, try
		 *  to group all the calls for a given dimensionality together or build different class instances for
		 *  queries of each dimensionality, etc.
		 *
		 *  \sa See some of the derived classes for example implementations of the pure virtual methods.
		 */
		class  KDTreeCapable
		{
		public:
			KDTreeCapable(); 			//!< Default ctor
			virtual ~KDTreeCapable(); 	//!< Dtor


			/** @name Public utility methods to query the KD-tree
				@{ */

			/** KD Tree-based search for the closest point (only ONE) to some given 2D coordinates.
			  *  This method automatically build the "KDTreeData" structure when:
			  *		- It is called for the first time
			  *		- The map has changed
			  *		- The KD-tree was build for 3D.
			  *
			  * \param x0  The X coordinate of the query.
			  * \param y0  The Y coordinate of the query.
			  * \param out_x The X coordinate of the found closest correspondence.
			  * \param out_y The Y coordinate of the found closest correspondence.
			  * \param out_dist_sqr The square distance between the query and the returned point.
			  *
			  * \return The index of the closest point in the map array.
			  *  \sa kdTreeClosestPoint3D, kdTreeTwoClosestPoint2D
			  */
			size_t kdTreeClosestPoint2D(
				float   x0,
				float   y0,
				float   	  &out_x,
				float   	  &out_y,
				float		  &out_dist_sqr
				) const;

		
		/** Like kdTreeClosestPoint2D, but just return the square error from some point to its closest neighbor.
			  */
			float kdTreeClosestPoint2DsqrError(
				float   x0,
				float   y0 ) const;

			/** KD Tree-based search for the TWO closest point to some given 2D coordinates.
			  *  This method automatically build the "KDTreeData" structure when:
			  *		- It is called for the first time
			  *		- The map has changed
			  *		- The KD-tree was build for 3D.
			  *
			  * \param x0  The X coordinate of the query.
			  * \param y0  The Y coordinate of the query.
			  * \param out_x1 The X coordinate of the first correspondence.
			  * \param out_y1 The Y coordinate of the first correspondence.
			  * \param out_x2 The X coordinate of the second correspondence.
			  * \param out_y2 The Y coordinate of the second correspondence.
			  * \param out_dist_sqr1 The square distance between the query and the first returned point.
			  * \param out_dist_sqr2 The square distance between the query and the second returned point.
			  *
			  *  \sa kdTreeClosestPoint2D
			  */
			void kdTreeTwoClosestPoint2D(
				float   x0,
				float   y0,
				float   	  &out_x1,
				float   	  &out_y1,
				float   	  &out_x2,
				float   	  &out_y2,
				float		  &out_dist_sqr1,
				float		  &out_dist_sqr2 ) const;

			/** KD Tree-based search for the N closest point to some given 2D coordinates.
			  *  This method automatically build the "KDTreeData" structure when:
			  *		- It is called for the first time
			  *		- The map has changed
			  *		- The KD-tree was build for 3D.
			  *
			  * \param x0  The X coordinate of the query.
			  * \param y0  The Y coordinate of the query.
			  * \param N The number of closest points to search.
			  * \param out_x The vector containing the X coordinates of the correspondences.
			  * \param out_y The vector containing the Y coordinates of the correspondences.
			  * \param out_dist_sqr The vector containing the square distance between the query and the returned points.
			  *
			  * \return The list of indices
			  *  \sa kdTreeClosestPoint2D
			  *  \sa kdTreeTwoClosestPoint2D
			  */
			std::vector<size_t> kdTreeNClosestPoint2D(
				float			x0,
				float			y0,
				size_t  N,
				std::vector<float>  &out_x,
				std::vector<float>  &out_y,
				std::vector<float>  &out_dist_sqr ) const;


			/** KD Tree-based search for the N closest point to some given 2D coordinates and returns their indexes.
			  *  This method automatically build the "KDTreeData" structure when:
			  *		- It is called for the first time
			  *		- The map has changed
			  *		- The KD-tree was build for 3D.
			  *
			  * \param x0  The X coordinate of the query.
			  * \param y0  The Y coordinate of the query.
			  * \param N The number of closest points to search.
			  * \param out_idx The indexes of the found closest correspondence.
			  * \param out_dist_sqr The square distance between the query and the returned point.
			  *
			  *  \sa kdTreeClosestPoint2D
			  */
			void kdTreeNClosestPoint2DIdx(
				float			x0,
				float			y0,
				size_t  N,
				std::vector<int>	&out_idx,
				std::vector<float>  &out_dist_sqr ) const;


			/** KD Tree-based search for the closest point (only ONE) to some given 3D coordinates.
			  *  This method automatically build the "KDTreeData" structure when:
			  *		- It is called for the first time
			  *		- The map has changed
			  *		- The KD-tree was build for 2D.
			  *
			  * \param x0  The X coordinate of the query.
			  * \param y0  The Y coordinate of the query.
			  * \param z0  The Z coordinate of the query.
			  * \param out_x The X coordinate of the found closest correspondence.
			  * \param out_y The Y coordinate of the found closest correspondence.
			  * \param out_z The Z coordinate of the found closest correspondence.
			  * \param out_dist_sqr The square distance between the query and the returned point.
			  *
			  * \return The index of the closest point in the map array.
			  *  \sa kdTreeClosestPoint2D
			  */
			size_t kdTreeClosestPoint3D(
				float   x0,
				float   y0,
				float   z0,
				float   	  &out_x,
				float   	  &out_y,
				float   	  &out_z,
				float		  &out_dist_sqr
				) const;

			/** KD Tree-based search for the N closest points to some given 3D coordinates.
			  *  This method automatically build the "KDTreeData" structure when:
			  *		- It is called for the first time
			  *		- The map has changed
			  *		- The KD-tree was build for 2D.
			  *
			  * \param x0  The X coordinate of the query.
			  * \param y0  The Y coordinate of the query.
			  * \param z0  The Z coordinate of the query.
			  * \param N The number of closest points to search.
			  * \param out_x The vector containing the X coordinates of the correspondences.
			  * \param out_y The vector containing the Y coordinates of the correspondences.
			  * \param out_z The vector containing the Z coordinates of the correspondences.
			  * \param out_dist_sqr The vector containing the square distance between the query and the returned points.
			  *
			  *  \sa kdTreeNClosestPoint2D
			  */
			void kdTreeNClosestPoint3D(
				float			x0,
				float			y0,
				float			z0,
				size_t  N,
				std::vector<float>  &out_x,
				std::vector<float>  &out_y,
				std::vector<float>  &out_z,
				std::vector<float>  &out_dist_sqr ) const;

			/** KD Tree-based search for the N closest point to some given 3D coordinates and returns their indexes.
			  *  This method automatically build the "KDTreeData" structure when:
			  *		- It is called for the first time
			  *		- The map has changed
			  *		- The KD-tree was build for 2D.
			  *
			  * \param x0  The X coordinate of the query.
			  * \param y0  The Y coordinate of the query.
			  * \param z0  The Z coordinate of the query.
			  * \param N The number of closest points to search.
			  * \param out_idx The indexes of the found closest correspondence.
			  * \param out_dist_sqr The square distance between the query and the returned point.
			  *
			  *  \sa kdTreeClosestPoint2D
			  */
			void kdTreeNClosestPoint3DIdx(
				float			x0,
				float			y0,
				float			z0,
				size_t  N,
				std::vector<int>	&out_idx,
				std::vector<float>  &out_dist_sqr ) const;

			void kdTreeNClosestPointsWith3DIdx(float x0,float	y0,float z0,std::vector<float>  &out_x, std::vector<float>  &out_y, std::vector<float>  &out_z, size_t  N, \
										std::vector<int> &out_idx, std::vector<float>  &out_dist_sqr ) const;

			/* @} */

		protected:
			/** To be called by child classes when KD tree data changes. */
			inline void kdtree_mark_as_outdated() const { m_KDTreeDataIsUpToDate = false; }

			/** @name Virtual methods that MUST be implemented by children classes of KDTreeCapable
			    @{ */

			/** Must return the number of data points */
			virtual size_t kdtree_get_point_count() const = 0;

			/** Must fill out the data points in "data", such as the i'th point will be stored in (data[i][0],...,data[i][nDims-1]). */
			virtual void kdtree_fill_point_data(ANNpointArray &data, const int nDims) const = 0;
			/** @} */

		private:
			/** Internal structure with a KD-tree representation.
			 */
			struct  TKDTreeData
			{
				/** Init the pointer to NULL. */
				TKDTreeData();

				/** Copy constructor: It actually does NOT copy the kd-tree, a new object will be created if required!   */
				TKDTreeData(const TKDTreeData &o);

				/** Copy operator: It actually does NOT copy the kd-tree, a new object will be created if required!  */
				TKDTreeData& operator =(const TKDTreeData &o);

				/** Free memory (if allocated) */
				~TKDTreeData();

				/** Free memory (if allocated)  */
				void clear();

				ANNkd_tree		*m_pDataTree;
				ANNpointArray 	m_DataPoints;
				ANNpoint 		m_QueryPoint;
				size_t 			m_nTreeSize;
				size_t 			m_nDim;
				size_t 			m_nk;
			};

			mutable TKDTreeData KDTreeData;

			mutable bool m_KDTreeDataIsUpToDate; //!< whether the KD tree needs to be rebuilt or not.

			void rebuild_kdTree(size_t nDims) const; //!< Rebuild, if needed the KD-tree for 2D (nDims=2), 3D (nDims=3), ... usage (asking the child class for the data points).


		};

#endif
