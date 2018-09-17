// PotentialModel.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"
#include "struct_fun.h"
#include "InputData.h"
#include "TrafficModel.h"

SLane LaneInfo; 
SObstacleInfo Obs;
SObstacle OBS;

int _tmain(int argc, _TCHAR* argv[])
{
	LaneSetting(LaneInfo);
	memset(&OBS,0,sizeof(SObstacle));
	fPoint obs1;
	obs1.x=-40;  
	obs1.y=3.75*2.5;
	ObsSetting(Obs,obs1,11);
	Obs.ObstacleID=Vehicle;
	OBS.Obs[0]=Obs;

	fPoint obs2;
	obs2.x=10;  
	obs2.y=3.75*2.5;
	ObsSetting(Obs,obs2,11);
	Obs.ObstacleID=Vehicle;
	OBS.Obs[1]=Obs;

	fPoint obs3;
	obs3.x=50;  
	obs3.y=3.75*2.5;
	ObsSetting(Obs,obs3,11);
	Obs.ObstacleID=Vehicle;
	OBS.Obs[2]=Obs;

	fPoint obs4;
	obs4.x=25;  
	obs4.y=3.75*1.5;
	ObsSetting(Obs,obs4,7);
	Obs.ObstacleID=Vehicle;
	OBS.Obs[3]=Obs;

	OBS.IsOK=TRUE;
	OBS.ObstacleNum=4;

	FrenetLaneMark LaneMark;
	memset(&LaneMark,0,sizeof(LaneMark));
	TrafficModel model;
	model.TrafficSetting(LaneInfo,OBS,LaneMark);
	model.veh.d=1.5*3.75;
	model.veh.s=10;
	model.veh.theta=0;
	model.veh.v=7;

	TreeNode node;
	node.s=130;
	node.d=2.5*3.75;

	//float lane_cost;
	float dynamic_cost;
	dynamic_cost=model.DynamicLaneRightCost(node,LaneMark);
	//lane_cost=model.LaneMarkCost(node,LaneMark);  
	int num=LaneInfo.LineR2.ValidNum;
	for (int i=0;i<num;i++)
	{
		//cout<<LaneInfo.LineR2.LinePt[i].x<<"  "<<LaneInfo.LineR2.LinePt[i].y<<endl;
		cout<<dynamic_cost<<endl;
        Sleep(10);
		//system("pause");
	}




	return 0;
}

