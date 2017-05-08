#ifndef C2DMAP_H
#define C2DMAP_H
#include <vector>

class C2DMap
{
public:
	enum Grid_S{Floor,Block,Sink,Unknown}; // Status of 2DMap for each Grid
	std::vector<std::vector<Grid_S> > m_vGrids; 
	std::vector<std::vector<int> > m_vGridsIndicator;
	unsigned int m_length;
	unsigned int m_width;
	
	unsigned int m_gridsize; // each grid stand for practical size
	unsigned int m_area_width;		// fixed area width 
	unsigned int m_area_length;		// fixed area length
	unsigned int m_total_grid;        // total grids number
	unsigned int m_offset_width;		// offset to make robot at the center of Area
	unsigned int m_offset_length;		// offset to make robot at the center of Area

	float m_lower_height;	// check area of this height
	float m_high_height;

	C2DMap(){}
	virtual ~C2DMap(){}
	virtual void  InitMap()=0;
	virtual void  UnInitMap()=0;
	virtual void  UpdateMap(void*)=0;
	virtual void  ClearMap()=0;
	virtual void  outStream(unsigned int lrow,unsigned int hrow, unsigned int lcol,unsigned int hcol, unsigned char* pBuf)=0;
	virtual void  outChangedGrid(unsigned char*pBuf)=0;
protected:
private:
};


#endif