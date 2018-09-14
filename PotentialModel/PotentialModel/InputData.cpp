#include "stdafx.h"
#include "InputData.h"






void LaneSetting(SLane &Lane)
{
	Lane.IsOK=TRUE;
	Lane.LaneNo=2;
	lineSetting(Lane.LineR2,0,2);
	lineSetting(Lane.LineR1,3.75,1);
	lineSetting(Lane.LineL1,7.5,1);
	lineSetting(Lane.LineL2,11.25,2);
	return;

}
void ObsSetting(SObstacleInfo &obs,fPoint &Center,int v)
{
	obs.Center=Center;
	obs.Heading=0;
	obs.v=v;
	obs.a=0;

	return;
}

void lineSetting(SLaneLine &line,float d,int LineType)
{
	int num=150/0.25;
	line.IsOK=TRUE;
    line.LaneLineClass = LineType;
	line.ValidNum=num;
	line.LaneLineNo=2;
	for(int i=0; i<num; i++)
	{
		line.LinePt[i].x=-50+0.25*i;
		line.LinePt[i].y=d;
	}
	return;

}
