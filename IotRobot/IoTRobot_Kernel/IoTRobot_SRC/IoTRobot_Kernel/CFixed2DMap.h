#ifndef CFIXED2DMAP_H
#define CFIXED2DMAP_H

#include "C2DMap.h"

//template <typename PointT>
//class CDynamicArea;

class CFixed2DMap : public C2DMap
{
public:
	CFixed2DMap();
	~CFixed2DMap();

	void InitMap();
	void UnInitMap();
	void UpdateMap(void* );
	void ClearMap();

	// 
	int getRow(float z);
	int getCol(float x);
	void outStream(unsigned int row,unsigned int col, unsigned char* pBuf);
	void outStream(unsigned int lrow,unsigned int hrow, unsigned int lcol,unsigned int hcol, unsigned char* pBuf);
	void outChangedGrid(unsigned char*pBuf);

	std::vector<unsigned int> m_Indices;
};

#endif