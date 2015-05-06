#ifndef GRID_2D_H
#define GRID_2D_H

#include <geometry_utils/GeometryUtils.h> 

class Grid2D
{
public: 
	unsigned char** grid2D_map;
	//lenght of each grid
	float grid_size;
	//diameter of fan
	float fan_R;
	//size of grid	
	int x_width;
	int y_width;
	//world rectangle
	double X_ORIGIN_IN_WORLD;
    double Y_ORIGIN_IN_WORLD;
	double X_LENGTH_WORLD;
	double Y_LENGTH_WORLD;

	Grid2D(float grid_size, float fan_R);
    ~Grid2D();

    //map visualization
	void plotMap();
	unsigned char* getMap();
	//coordinate convertion
	float findCellCenterCoord(int grid, char axis);
	int findGrid(float value, char axis);
	geometry_utils::Vec3 findGrids(geometry_utils::Vec3 values);
	//boundary checking
	bool checkFan(geometry_utils::Vec3 fan);
	bool checkPoint(geometry_utils::Vec3 pt);
	//map generation
	void markFanOnGrid(unsigned char** grid2D, geometry_utils::Vec3 fan);
	unsigned char** makeGrid2D(geometry_utils::Vec3 fan1, geometry_utils::Vec3 fan2, bool output_flag);

private:
	float dist(geometry_utils::Vec3 pt1, geometry_utils::Vec3 pt2);
};

#endif