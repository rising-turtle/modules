#include "CDynamicArea.h"

int CDynamicArea::m_s_CellSize=4;

#define UPPER_ASS(Upp,V){if(V>Upp) Upp=V;}

CDynamicArea::CDynamicArea():m_dynamic_cell_map(new pcl::PointCloud<pcl::PointXYZRGB>)
{
	m_lower_x=0;
	m_lower_y=0;
	m_lower_z=0;
	m_upper_x=0;
	m_upper_y=0;
	m_upper_z=0;

	m_upper_y_topview=0.4;
	m_lower_y_topview=-0.4;
	m_upper_y_topview_cell=2;
	m_lower_y_topview_cell=0;
	m_bNeedTopView=false;
	initArea();
}
CDynamicArea::CDynamicArea(float l_x,float u_x,float l_y,float u_y,float l_z,float u_z):m_dynamic_cell_map(new pcl::PointCloud<pcl::PointXYZRGB>),\
							m_lower_x(l_x),m_upper_x(u_x),m_lower_y(l_y),m_upper_y(u_y),m_lower_z(l_z),m_upper_z(u_z)
{
	initArea();
}
CDynamicArea::~CDynamicArea(){}

void CDynamicArea::initArea()
{
	m_x_offset=(int)(m_lower_x*100-1);
	m_y_offset=(int)(m_lower_y*100-1);
	m_z_offset=(int)(m_lower_z*100-1);

	m_x_range=(int)((m_upper_x-m_lower_x)*100);
	m_y_range=(int)((m_upper_y-m_lower_y)*100);
	m_z_range=(int)((m_upper_z-m_lower_z)*100);

	m_x_cell=m_x_range>>2; // cell_size = 4
	m_y_cell=m_y_range>>2;
	m_z_cell=m_z_range>>2;

	m_x_step=m_y_cell*m_z_cell;
	m_y_step=m_z_cell;
	m_all_cells=m_x_cell*m_y_cell*m_z_cell;

	m_dynamic_cells.resize(m_all_cells);
	m_valid_flag.resize(m_all_cells,false);

	//cout<<"dynamic_cells.size="<<m_all_cells<<endl;
	//cout<<"m_x_cell:"<<m_x_cell<<endl;
	//cout<<"m_y_cell:"<<m_y_cell<<endl;
	//cout<<"m_z_cell:"<<m_z_cell<<endl;
	//cout<<"m_x_offset:"<<m_x_offset<<endl;
	//cout<<"m_y_offset:"<<m_y_offset<<endl;
	//cout<<"m_z<<offset:"<<m_z_offset<<endl;


	if(m_bNeedTopView)
	{
		initTopView();
	}
}
void CDynamicArea::unitArea()
{	
	m_dynamic_cells.swap(std::vector<boost::shared_ptr<pcl::PointXYZRGB> >());
	m_dynamic_cell_map->points.clear();
	m_valid_flag.resize(0,false);
}

void CDynamicArea::reset(float l_x,float u_x,float l_y,float u_y,float l_z,float u_z)
{
	unitArea();
	static float error_tor=0.08;
	m_lower_x=l_x;//-error_tor;
	m_lower_y=l_y;//-error_tor;
	m_lower_z=l_z;//-error_tor;
	m_upper_x=u_x+error_tor;
	m_upper_y=u_y+error_tor;
	m_upper_z=u_z+error_tor;
	initArea();
}
int CDynamicArea::getIndexCell(float& x,float& y, float& z)
{
	if(_isnan(x) || _isnan(y) || _isnan(z))
		return -1;
	if(x>=m_upper_x || y>=m_upper_y || z>=m_upper_z \
		|| x<m_lower_x || y<m_lower_y || z<m_lower_z )
		return -1;
	int lx = floor(( x*100 - m_x_offset)+0.5);
	lx>>=2;//divide by cell_size
	int ly = floor(( y*100 - m_y_offset)+0.5); 
	ly>>=2;//divide by cell_size
	int lz = floor(( z*100 - m_z_offset)+0.5); 
	lz>>=2;//divide by cell_size

	if(lx >= m_x_cell || ly>= m_y_cell || lz >= m_z_cell \
		|| lx<0 || ly<0 || lz<0)
	{
		cout<<"exceed range in CDynamic::getIndex() at "<<"("<<x<<","<<y<<","<<z<<")"<<endl;
		if(lx>=m_x_cell)
			cout<<"lx="<<lx<<",x_cell="<<m_x_cell<<endl;
		if(ly>=m_y_cell)
			cout<<"ly="<<ly<<",y_cell="<<m_y_cell<<endl;
		if(lz>=m_z_cell)
			cout<<"lz="<<lz<<",z_cell="<<m_z_cell<<endl;
		return -1;
	}

	if(m_bNeedTopView)
	{
		if(ly>=m_lower_y_topview_cell && ly<m_upper_y_topview_cell)
			UPPER_ASS(m_vGridsIndicator[lx][lz],ly);
	}

	return (lx*m_x_step + ly*m_y_step + lz);
}

void CDynamicArea::FromPC2Cell(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> >& cloud) //FromPC2Cell
{
	int N = cloud->points.size();
	static boost::dynamic_bitset<> cur_frame;
	cur_frame.resize(m_all_cells,false);
	cur_frame.reset();
	//int num=0;
	//int nDelNum1=0,nDelNum2=0,nDelNum3=0;

	//cout<<"PC2Cell PC SIZE: "<<cloud->points.size()<<endl;
	for(size_t i=0;i<cloud->points.size();i++)
	{
		pcl::PointXYZRGB& sp = cloud->points[i];
		int index = this->getIndexCell(sp.x,sp.y,sp.z);
		if(index >=0 )
		{
			if(this->m_valid_flag[index])
			{/*pc3Dmap->m_global_cells[index].get()!=NULL*/ // this is already painted
				//nDelNum2++;
				continue;
			}// not flush this point using the new point	
			boost::shared_ptr<pcl::PointXYZRGB> p(new pcl::PointXYZRGB);
			p->x = sp.x; p->y = sp.y; p->z = sp.z; 
			p->r = sp.r; p->g = sp.g; p->b = sp.b;
			if(!cur_frame[index])
			{
				//num++;
				cur_frame.set(index);
				this->m_dynamic_cells[index] = p;
				this->m_dynamic_cell_map->points.push_back(sp);
			}
			
		}
		
	}
	//cout<<"!!! N of points into Node: "<< pc->points.size()<<endl;
	this->m_valid_flag |= cur_frame; // 
	/*cout<<"num of inserted="<<num<<endl;
	cout<<"del1="<<nDelNum1<<"; del2="<<nDelNum2<<"; del3="<<nDelNum3<<endl;
	cout<<"m_dynamic_cell_map.size="<<this->m_dynamic_cell_map->points.size();*/
}

void CDynamicArea::initTopView()
{
	cout<<"initTopView!"<<endl;
	m_vGridsIndicator.resize(m_x_cell);
	for(int i=0;i<m_x_cell;i++)
		m_vGridsIndicator[i].resize(m_z_cell,-1);
	int m_topview_y_cell=(int)((m_upper_y_topview-m_lower_y_topview)*100);// actually it is 20 for [-0.4,0.4]
	m_topview_cells.resize(m_x_cell*m_z_cell*20);

	// find index along y-axis according to [-0.4,0.4]
	m_upper_y_topview_cell=(int)((m_upper_y_topview-m_lower_y)*100);
	m_lower_y_topview_cell=(int)((m_lower_y_topview-m_lower_y)*100);
	if(m_upper_y_topview_cell<=m_lower_y_topview_cell)
		cout<<"error at topview y-range:"<<m_lower_y_topview_cell<<"-"<<m_upper_y_topview_cell<<endl;
	while(m_upper_y_topview_cell-m_lower_y_topview_cell>20)
		m_upper_y_topview_cell--;
	
	m_bIsTopViewAvailable=false;
}
void CDynamicArea::generateTopView()
{
	int lx,ly,lz;
	
	m_vGrids.resize(m_x_cell);
	for(int i=0;i<m_x_cell;i++)
		m_vGrids[i].resize(m_z_cell);

	for(lx=0;lx<m_x_cell;lx++)	
		for(lz=0;lz<m_z_cell;lz++)
		{
			ly=m_vGridsIndicator[lx][lz];
			if(ly==-1)
			{
				m_vGrids[lx][lz]=Grid_S::UnKnown;
			}
			else if(ly<m_lower_y_topview_cell)
			{
				m_vGrids[lx][lz]=Grid_S::Sink;
			}
			else if(ly>=m_lower_y_topview_cell && ly<= m_lower_y_topview_cell+2)//m_upper_y_topview_cell)
			{
				m_vGrids[lx][lz]=Grid_S::Floor;
			}
			else
				m_vGrids[lx][lz]=Grid_S::Block;
		}
		m_bIsTopViewAvailable=true;
}
void CDynamicArea::unitTopView()
{
	for(int i=0;i<m_x_cell;i++)
	{
		m_vGrids[i].clear();
		m_vGridsIndicator[i].clear();
	}
	m_vGrids.clear();
	m_vGridsIndicator.clear();
	m_bIsTopViewAvailable=false;
}