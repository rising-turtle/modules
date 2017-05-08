#ifndef SIMPLEPOINTS_H
#define SIMPLEPOINTS_H

#include "KDTreeCapable.h"

class CSimplePoints : public CKDTreeCapable
{
public:
	CSimplePoints();
	~CSimplePoints();

	/** @name Virtual methods that MUST be implemented by children classes of KDTreeCapable
	@{ */
	/** Must return the number of data points */
	virtual size_t kdtree_get_point_count() const;

	/** Must fill out the data points in "data", such as the i'th point will be stored in (data[i][0],...,data[i][nDims-1]). */
	virtual void kdtree_fill_point_data(ANNpointArray &data, const int nDims) const;

};

#endif