#include <time.h>
#include <cmath>
#include <stdlib.h>
#include "TinySlam.h"

#include "./tinySlam/CoreSLAM.h"

#define TEST_FILENAME "test_lab2"
#define TEST_MAX_SCANS 5000
#define TEST_SCAN_SIZE 682

namespace {
	int ts_read_scans(char *filename, ts_sensor_data_t *sensor_data)
	{
		FILE *input;
		char *str, line[4000];
		int i, nbscans = 0;
		ts_sensor_data_t *sd;

		input = fopen(filename, "rt");
		if(input==NULL){
				printf("failed to open file!\n");
				return -1;
		}
		do {    
			sd = &sensor_data[nbscans];
			// Read the scan
			str = fgets(line, 4000, input);
			if (str == NULL) break;
			str = strtok(str, " ");
			sscanf(str, "%u", &sd->timestamp);
			str = strtok(NULL, " ");
			str = strtok(NULL, " ");
			sscanf(str, "%d", &sd->q1);
			str = strtok(NULL, " ");
			sscanf(str, "%d", &sd->q2);
			for (i = 0; i < 21; i++)
				str = strtok(NULL, " ");
			for (i = 0; i < TEST_SCAN_SIZE; i++) {
				if (str) {
					sscanf(str, "%d", &sd->d[i]);
					str = strtok(NULL, " ");
				} else sd->d[i] = 0;
			}
			nbscans++;
		} while (1);
		fclose(input);
		return nbscans;
	}

};

ts_map_t trajectory, t_map, loop_map, map_scans;
ts_sensor_data_t sensor_data[TEST_MAX_SCANS];

CTinySlam::CTinySlam(){}
CTinySlam::~CTinySlam(){}

// read log file into ts_sensor_data_t struct
int CTinySlam::readSick511(string fdata ){
	ifstream inf(fdata.c_str());
	if(!inf.is_open()){
		cout<<"failed to open file : "<<fdata<<endl;
		return -1;
	}
	int index=0;
	char line[4096];
	int N;
	ts_sensor_data_t * psd;
	while(inf.getline(line,4096) && index < TEST_MAX_SCANS){
		strtok(line," ");
		N=(int)(atof(strtok(NULL," ")));
		psd=&sensor_data[index++];
		//psd->position = TS_DIRECTION_FORWARD;
		psd->q1=0;
		psd->q2=0;
		psd->psidot=0;
		psd->v=0;
		for(int i=0;i<N;i++){
			psd->d[i] = atof(strtok(NULL," "));
		}
	}
	if(index>=TEST_MAX_SCANS){
		cout<<"scan number surpass TEST_MAX_SCANS!"<<endl;
	}
	return index;
}
void CTinySlam::runSick511(string fdata){
	int maxscans = readSick511(fdata);
	if(maxscans<0){
		cout<<"input error!"<<endl;
		return ;
	}
	int nb_loops, loop_start,x,y,quality;
	int loop_end[100];
	nb_loops = 1;
	loop_end[nb_loops - 1] = maxscans;
	
	ts_position_t startpos, loop_startpos, loop_endpos, position;
	ts_robot_parameters_t params;
	ts_laser_parameters_t laser_params;
	ts_state_t state;

	params.r = 0.077;
	params.R = 0.165;
	params.inc = 2000;
	params.ratio = 1.0;

	laser_params.offset = 145;
	laser_params.scan_size = TEST_SCAN_SIZE;
	laser_params.angle_min = -120;
	laser_params.angle_max = +120;
	laser_params.detection_margin = 70;
	laser_params.distance_no_detection = 4000;

	startpos.x = 0.5 * TS_MAP_SIZE / TS_MAP_SCALE;
	startpos.y = 0.6 * TS_MAP_SIZE / TS_MAP_SCALE; 
	startpos.theta = 0;

	ts_map_init(&t_map);
	ts_map_init(&trajectory);
	ts_map_init(&loop_map);
	
	loop_start = 0;
	loop_startpos = startpos;
	
	for (int loop = 0; loop != nb_loops; loop++) { 
		ts_state_init(&state, &t_map, &params, &laser_params, &loop_startpos, 100, 20, 600, TS_DIRECTION_FORWARD);
		for (int nbscans = loop_start; nbscans != loop_end[loop]; nbscans++) {
			ts_iterative_map_building(&sensor_data[nbscans], &state);
			printf("#%d : %lg %lg %lg\n", nbscans, state.position.x, state.position.y, state.position.theta);
			if (nbscans == 50) 
				loop_map = t_map;

			x = (int)floor(state.position.x * TS_MAP_SCALE + 0.5);
			y = ((int)floor(state.position.y * TS_MAP_SCALE + 0.5));
			if (x >= 0 && y >= 0 && x < TS_MAP_SIZE && y < TS_MAP_SIZE)
				trajectory.map[y * TS_MAP_SIZE + x] = 0;
		}
		cout<<"succeed!"<<endl;
	}
	return ;
}
void CTinySlam::run(string fdata){
	ts_position_t startpos, loop_startpos, loop_endpos, position;
	char filename[256], test_filename[256];
	int i, x, y;
	int quality;
	int nbscans, maxscans;
	ts_robot_parameters_t params;
	ts_laser_parameters_t laser_params;
	ts_state_t state;
	int loop, loop_start, nb_loops, loop_end[2];

	// record time performance
	FILE* per;
	double start_slam;
	int frame;

	// 打开读入文件
	strcpy(test_filename,fdata.c_str());
	maxscans=ts_read_scans(test_filename,sensor_data);

	// 设置loop参数？？
	nb_loops = 1;
	loop_end[nb_loops - 1] = maxscans;

	params.r = 0.077;
	params.R = 0.165;
	params.inc = 2000;
	params.ratio = 1.0;

	laser_params.offset = 145;
	laser_params.scan_size = TEST_SCAN_SIZE;
	laser_params.angle_min = -120;
	laser_params.angle_max = +120;
	laser_params.detection_margin = 70;
	laser_params.distance_no_detection = 4000;

	startpos.x = 0.5 * TS_MAP_SIZE / TS_MAP_SCALE;
	startpos.y = 0.6 * TS_MAP_SIZE / TS_MAP_SCALE; 
	startpos.theta = 0;

	// 初始化地图
	ts_map_init(&t_map);
	ts_map_init(&trajectory);
	ts_map_init(&loop_map);

	loop_start = 0;
	loop_startpos = startpos;

	// 记录slam时间
	per = fopen("per.log","wb");
	frame = 0;
	if(per==NULL)
		printf("failed to open file: per.log\n");
	fprintf(per,"Frame \tTime\n");
	
	// 循环读入文件内容，模拟slam过程
	for (loop = 0; loop != nb_loops; loop++) { 
		ts_state_init(&state, &t_map, &params, &laser_params, &loop_startpos, 100, 20, 600, TS_DIRECTION_FORWARD);
		for (nbscans = loop_start; nbscans != loop_end[loop]; nbscans++) {
			start_slam = clock();
			ts_iterative_map_building(&sensor_data[nbscans], &state);
			fprintf(per,"%d\t%lf\n",++frame,clock()-start_slam);
			printf("#%d : %lg %lg %lg\n", nbscans, state.position.x, state.position.y, state.position.theta);

			if (nbscans == 50) 
				loop_map = t_map;

			x = (int)floor(state.position.x * TS_MAP_SCALE + 0.5);
			y = ((int)floor(state.position.y * TS_MAP_SCALE + 0.5));
			if (x >= 0 && y >= 0 && x < TS_MAP_SIZE && y < TS_MAP_SIZE)
				trajectory.map[y * TS_MAP_SIZE + x] = 0;
		}

		fclose(per);
		// Record the map
		sprintf(filename, "%s_loop%d_forward.pgm", test_filename, loop);
		ts_save_map_pgm(&t_map, &trajectory, filename, TS_MAP_SIZE, TS_MAP_SIZE);

		printf("Try to close the loop...\n");

		position = state.position;
		position.x += state.laser_params.offset * cos(position.theta * M_PI / 180);
		position.y += state.laser_params.offset * sin(position.theta * M_PI / 180);
		loop_endpos = ts_close_loop_position(&state, &sensor_data[loop_end[loop] - 1], &loop_map, &position, &quality);
		loop_endpos.x -= state.laser_params.offset * cos(loop_endpos.theta * M_PI / 180);
		loop_endpos.y -= state.laser_params.offset * sin(loop_endpos.theta * M_PI / 180);

		printf("Loop close point : %lg %lg %lg (%d)\n", loop_endpos.x, loop_endpos.y, loop_endpos.theta, quality);
		printf("Deviation is : %lg mm, %lg degrees\n", ts_distance(&state.position, &loop_endpos), fabs(state.position.theta - loop_endpos.theta)); 

		t_map = loop_map;
		ts_map_init(&trajectory);
	}
}