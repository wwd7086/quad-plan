/*
 * Copyright (c) 2008, Maxim Likhachev
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Pennsylvania nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include <cmath>
#include <cstring>
#include <iostream>
#include <string>

using namespace std;

#include <sbpl/headers.h>
#include "grid2D.h"

enum EnvironmentType
{
    INVALID_ENV_TYPE = -1, ENV_TYPE_2D, ENV_TYPE_2DUU, ENV_TYPE_XYTHETA, ENV_TYPE_XYTHETAMLEV, ENV_TYPE_ROBARM,

    NUM_ENV_TYPES
};

enum MainResultType
{
    INVALID_MAIN_RESULT = -1,

    MAIN_RESULT_SUCCESS = 0,
    MAIN_RESULT_FAILURE = 1,
    MAIN_RESULT_INSUFFICIENT_ARGS = 2,
    MAIN_RESULT_INCORRECT_OPTIONS = 3,
    MAIN_RESULT_UNSUPPORTED_ENV = 4,

    NUM_MAIN_RESULTS
};

/*******************************************************************************
 * PrintUsage - Prints the proper usage of the sbpl test executable.
 *
 * @param argv The command-line arguments; used to determine the name of the
 *             test executable.
 *******************************************************************************/
void PrintUsage(char *argv[])
{
    printf("USAGE: %s [-s] [--env=<env_t>] [--planner=<planner_t>] [--search-dir=<search_t>] <cfg file> [mot prims]\n",
           argv[0]);
    printf("See '%s -h' for help.\n", argv[0]);
}

/*******************************************************************************
 * CheckIsNavigating
 * @brief Returns whether the -s option is being used.
 *
 * @param numOptions The number of options passed through the command line
 * @param argv The command-line arguments
 * @return whether the -s option was passed in on the cmd line
 *******************************************************************************/
bool CheckIsNavigating(int numOptions, char** argv)
{
    for (int i = 1; i < numOptions + 1; i++) {
        if (strcmp(argv[i], "-s") == 0) {
            return true;
        }
    }
    return false;
}

/*******************************************************************************
 * CheckSearchDirection -
 * @brief Returns the search direction being used
 *
 * @param numOptions The number of options passed through the command line
 * @param argv The command-line arguments
 * @return A string representing the search direction; "backward" by default
 ******************************************************************************/
std::string CheckSearchDirection(int numOptions, char** argv)
{
    int optionLength = strlen("--search-dir=");
    for (int i = 1; i < numOptions + 1; i++) {
        if (strncmp("--search-dir=", argv[i], optionLength) == 0) {
            std::string s(&argv[i][optionLength]);
            return s;
        }
    }
    return std::string("backward");
}

/*******************************************************************************
 * CheckEnvironmentType
 * @brief Returns the environment being used
 *
 * @param numOptions The number of options passed through the command line
 * @param argv The command-line arguments
 * @return A string denoting the environment type; "xytheta" by default
 *******************************************************************************/
std::string CheckEnvironmentType(int numOptions, char** argv)
{
    int optionLength = strlen("--env=");
    for (int i = 1; i < numOptions + 1; i++) {
        if (strncmp("--env=", argv[i], optionLength) == 0) {
            std::string s(&argv[i][optionLength]);
            return s;
        }
    }
    return std::string("xytheta");
}

/*******************************************************************************
 * CheckPlannerType - Checks for a planner specifier passed in through the
 *                    command line. This determines what planner to run in
 *                    the example. If none is found, ARA* is assumed.
 *
 * @param numOptions The number of options passed through the command line
 * @param argv The command-line arguments
 * @return A string denoting the planner type; "arastar" by default
 ******************************************************************************/
std::string CheckPlannerType(int numOptions, char** argv)
{
    int optionLength = strlen("--planner=");
    for (int i = 1; i < numOptions + 1; i++) {
        if (strncmp("--planner=", argv[i], optionLength) == 0) {
            std::string s(&argv[i][optionLength]);
            return s;
        }
    }
    return std::string("arastar");
}


int main()
{
	float grid_size = 0.2;
	float fan_R = 0.5;

	bool forwardSearch = true;
    int bRet = 0;
    double allocated_time_secs = 100.0; // in seconds
    double initialEpsilon = 3.0;
	unsigned char obsthresh = 1;
    MDPConfig MDPCfg;
    bool bsearchuntilfirstsolution = false;
    bool bforwardsearch = forwardSearch;

	float fan1[2] = {-0.2, -0.35};
	float fan2[2] = {1.8, 2.45};
	int startx = 0;
	int starty = 0;
	int goalx = 2;
	int goaly = 2;

    // Initialize Environment (should be called before initializing anything else)
	Grid2D grid2D = Grid2D(grid_size, fan_R); // Create grid2D map object with desired grid size and fan R
    EnvironmentNAV2D environment_nav2D; // Create Environment nav 2D object

	// Adding fan positions
	grid2D.makeGrid2D(fan1, fan2, true);

	unsigned char mapdata[grid2D.x_width*grid2D.y_width];
	for (int y=0;y<grid2D.y_width;y++)
	{	
		for (int x=0;x<grid2D.x_width;x++)
		{	
			mapdata[x + y * grid2D.x_width] = grid2D.grid2D_map[x][y];
		}
	}
	environment_nav2D.InitializeEnv(grid2D.x_width, grid2D.y_width, mapdata, startx, starty, goalx, goaly, obsthresh);
	//environment_nav2D.SetConfiguration(grid2D.x_width, grid2D.y_width, mapdata, startx, starty, goalx, goaly);

	// Initialize MDP Info
    if (!environment_nav2D.InitializeMDPCfg(&MDPCfg)) {
        printf("ERROR: InitializeMDPCfg failed\n");
        throw new SBPL_Exception();
    }

    // plan a path
    vector<int> solution_stateIDs_V;

    SBPLPlanner* planner = NULL;
	printf("Initializing ADPlanner...\n");
	planner = new ADPlanner(&environment_nav2D, bforwardsearch);

    // set search mode
    planner->set_search_mode(bsearchuntilfirstsolution);

    if (planner->set_start(MDPCfg.startstateid) == 0) {
        printf("ERROR: failed to set start state\n");
        throw new SBPL_Exception();
    }

    if (planner->set_goal(MDPCfg.goalstateid) == 0) {
        printf("ERROR: failed to set goal state\n");
        throw new SBPL_Exception();
    }

    planner->set_initialsolution_eps(initialEpsilon);

    printf("start planning...\n");
    bRet = planner->replan(allocated_time_secs, &solution_stateIDs_V);
    printf("done planning\n");
    std::cout << "size of solution=" << solution_stateIDs_V.size() << std::endl;

    environment_nav2D.PrintTimeStat(stdout);

    const char* sol = "sol.txt";
    FILE* fSol = fopen(sol, "w");
    if (fSol == NULL) {
        printf("ERROR: could not open solution file\n");
        throw new SBPL_Exception();
    }

    int x, y;
    for (unsigned int i = 0; i < solution_stateIDs_V.size(); i++) {
        environment_nav2D.PrintState(solution_stateIDs_V[i], true, fSol);
	environment_nav2D.GetCoordFromState(solution_stateIDs_V[i], x, y);
	std::cout << "X: " << x << "  Y: " << y << std::endl;
    }
    fclose(fSol);

    environment_nav2D.PrintTimeStat(stdout);

    //print a path
    if (bRet) {
        //print the solution
        printf("Solution is found\n");
    }
    else {
        printf("Solution does not exist\n");
    }

    fflush(NULL);

    delete planner;

    return bRet;
}
