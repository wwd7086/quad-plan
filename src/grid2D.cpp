#include <cmath>
#include <cstring>
#include <string>
#include <sbpl_plan/grid2D.h>
#include <ros/ros.h>

//-------------------constructor--------------------------------------------
Grid2D::Grid2D(float grid_size, float fan_R)
{	
	this->grid_size = grid_size;
	this->fan_R = fan_R;

	X_ORIGIN_IN_WORLD = -1.05;
    Y_ORIGIN_IN_WORLD = -1.44;
	X_LENGTH_WORLD = 2.9;
	Y_LENGTH_WORLD = 3.94;

	x_width = ceil(X_LENGTH_WORLD/grid_size);
	y_width = ceil(Y_LENGTH_WORLD/grid_size);
	
	this->grid2D_map = new unsigned char*[x_width];
	for (int x = 0; x < x_width; x++) {
        grid2D_map[x] = new unsigned char[y_width];
    }

	for (int y = 0; y < y_width; y++) {
        for (int x = 0; x < x_width; x++) {
            grid2D_map[x][y] = 0;
        }
    }
}

//--------------------Local Functions -------------------------
void Grid2D::plotMap()
{
	for (int y = 0; y < y_width; y++) {
		for (int x = 0; x < x_width; x++) {
			ROS_INFO_STREAM((int)grid2D_map[x][y]<<" ");
		}
	}
}

unsigned char* Grid2D::getMap()
{
	unsigned char* mapdata= new unsigned char[x_width*y_width];
	for (int y=0; y<y_width; y++)
	{	
		for (int x=0;x<x_width;x++)
			mapdata[x + y * x_width] = grid2D_map[x][y];
	}
	return mapdata;
}

float Grid2D::findCellCenterCoord(int grid, char axis)
{
	float coord;
	if (axis=='X') coord = (grid + 0.5) * grid_size + X_ORIGIN_IN_WORLD;
	else if (axis=='Y')	coord = (grid + 0.5) * grid_size + Y_ORIGIN_IN_WORLD;
	else ROS_INFO("Wrong axis input for calculating center coordinates.");
	return coord;
}

bool Grid2D::checkFan(geometry_utils::Vec3 fan)
{
	if ((fan.x()<X_ORIGIN_IN_WORLD) or (fan.x()>X_ORIGIN_IN_WORLD+X_LENGTH_WORLD)){
		ROS_INFO("Invalid fan position");
		return false;
	}
	if ((fan.y()<Y_ORIGIN_IN_WORLD) or (fan.y()>Y_ORIGIN_IN_WORLD+Y_LENGTH_WORLD)){
		ROS_INFO("Invalid fan position");
		return false;
	}
	return true;
}

bool Grid2D::checkPoint(geometry_utils::Vec3 pt)
{
	if ((pt.x()<X_ORIGIN_IN_WORLD) or (pt.x()>X_ORIGIN_IN_WORLD+X_LENGTH_WORLD)){
		return false;
	}
	if ((pt.y()<Y_ORIGIN_IN_WORLD) or (pt.y()>Y_ORIGIN_IN_WORLD+Y_LENGTH_WORLD)){
		return false;
	}
	return true;
}

int Grid2D::findGrid(float value, char axis)
{
	int cell;
	if (axis == 'X')
	{
		cell = floor((value - X_ORIGIN_IN_WORLD)/grid_size);
	}
	else if (axis == 'Y')
	{
		cell = floor((value - Y_ORIGIN_IN_WORLD)/grid_size);
	}
	else 
	{
		ROS_INFO("None Valid axis, type in 'X' or 'Y' instead. ");
		return 0;
	}
	return cell;
}

geometry_utils::Vec3 Grid2D::findGrids(geometry_utils::Vec3 values)
{
	geometry_utils::Vec3 result(findGrid(values.x(),'X'),
								findGrid(values.y(),'Y'),
								0);
	return result;
}

float Grid2D::dist(geometry_utils::Vec3 pt1, geometry_utils::Vec3 pt2)
{
	return sqrt((pt1.x()-pt2.x())*(pt1.x()-pt2.x()) + (pt1.y()-pt2.y())*(pt1.y()-pt2.y()));
}

void Grid2D::markFanOnGrid(unsigned char** grid2D, geometry_utils::Vec3 fan)
{
 	int fan_x = findGrid(fan.x(), 'X');
	int fan_y = findGrid(fan.y(), 'Y');
	int span_x = ceil(fan_R/grid_size);
	int span_y = ceil(fan_R/grid_size);

	for (int i=fan_x-span_x;i<fan_x+span_x;i++){
		for(int j=fan_y-span_y;j<fan_y+span_y;j++){
			if ((i>=0)&&(i<ceil(X_LENGTH_WORLD/grid_size))&&(j>=0)&&(j<ceil(Y_LENGTH_WORLD/grid_size))){
				geometry_utils::Vec3 cellCenter(findCellCenterCoord(i,'X'),
												findCellCenterCoord(j,'Y'),
												0); 
				if ((dist(fan, cellCenter)-grid_size*1.4/2)<fan_R){
					grid2D[i][j] = 1;
				}
			}
		}
	}
	return;
}

unsigned char** Grid2D::makeGrid2D(geometry_utils::Vec3 fan1, geometry_utils::Vec3 fan2, bool output_flag)
{

	//fans
	if (checkFan(fan1) && checkFan(fan2)){
		markFanOnGrid(grid2D_map, fan1);
		markFanOnGrid(grid2D_map, fan2);

		if(output_flag){
			//Output Grid Map
			plotMap();
		}
	}
	else{
			ROS_INFO("Error detected, grid is not generated.");
	}

	return grid2D_map;
}

//--------------------Distructor-------------------
Grid2D::~Grid2D()
{
	if (grid2D_map != NULL) {
		int x_width = ceil(X_LENGTH_WORLD/grid_size);
        for (int x = 0; x < x_width; x++) {
            if (grid2D_map[x] != NULL) delete[] grid2D_map[x];
        }
        delete[] grid2D_map;
    }
}


//Example
//float fan1[2] = {-0.2, -0.35};
//float fan2[2] = {1.8, 2.45};
//Grid2D grid2D = Grid2D(0.1,0.5);
//grid2D.makeGrid2D(fan1, fan2, true);

