#pragma once
#include "stdafx.h"


#define	MaxSpeed		18

#define LinePtNumMax	600		// 单车道线点数上限
#define CARLENGTH 3.8	//车长
#define CARHALFWIDTH 1.1	//车宽
#define Step	0.25   // 分辨率，单位m
#define ObstacleMaxNum	100		// 障碍数量上限
#define N_PARAM		4		//	现定的曲线多项式项数		



// 连续坐标系中的点坐标
class fPoint
{
public:
	float x;
	float y;
	fPoint(){
	}
	~fPoint(){
	}
	fPoint(float a,float b){
		x = a;y = b ;
	}
	float Pt2PtDist(fPoint &tmp)
	{
		return sqrt(pow((x-tmp.x),2)+pow((y-tmp.y),2));
	}
};

// 车道线
class SLaneLine
{
public:
	BOOL			IsOK;
	int				LaneLineNo;
	int				LaneLineClass;			// 0 - 可穿越； 1 - 不可穿越； 3 - 跟踪线
	int				ValidNum;
	fPoint			LinePt[LinePtNumMax];
	//vector<fPoint>      LinePt;
	~SLaneLine(){}
	SLaneLine()
	{
		IsOK = FALSE;
		LaneLineNo = 9;
		LaneLineClass = 0;
		ValidNum = 0;
	}


};


// 障碍物类型
enum EObsMotionType
{	
	Static = 0,	   // static. 
	Moving = 1,	   // moving.
	TooLow = -5,     // the height difference is too small. 
	Unkown = -4,     // unkown motion type. 
	FewPts = -3,     // the obstacle contains few points. 
	UnMatch = -2,    // the current obstacle cannot find corresponding points in previous frame within some meters. 
	BadICP = -1      // icp results is low quality.
};

enum EObstacleClass
{
	// 0 un_know 1 vehicle 2 Motorcycle 3 Truck 4 Pedestrian 9 Bicycle.
	unknow = 0,
	Vehicle = 1, 
	Motorcycle = 2, 
	Truck = 3,       
	Human = 4,  
	warning = 5,	//	警示牌
	Cone = 6,	//	锥型标	   
	Bicycle = 9,  	
};

// 车道结构
class SLane
{
public:
	BOOL			IsOK;			//TRUE-可用，FALSE-不可用
	BOOL            IsVirtual;		//
	BOOL            IsSplit;
	int             CurRoadPriority;//
	int             LaneNo;	    	//当前车体所在绝对车道号，从最右车道为0起，向左依次累加
	double			LaneWidth;		//车道宽度
	double          SplitDis;
	SLaneLine		LineR2;			//车体参考点右侧第二根
	SLaneLine		LineR1;			//车体参考点右侧第一根
	SLaneLine		LineL1;			//车体参考点左侧第一根
	SLaneLine		LineL2;			//车体参考点左侧第二根
	SLaneLine       GlobalGuideLine;//全局引导线
	double			LaneMaxSpeed;	//当前帧车道行驶的最高速度
	SLane(){
		IsSplit = FALSE;
		IsOK = FALSE;
		IsVirtual = FALSE;
		CurRoadPriority = 0;
		LaneNo = 9;
		LaneWidth = 0;
		SplitDis = 1000.0;
		LaneMaxSpeed = MaxSpeed;
	}

};

struct SObstacleInfo
{
	EObsMotionType	 MotionState;	//	障碍物状态()
	fPoint 			 Center;		//	障碍物BOX的中心点在局部坐标系中的坐标
	fPoint 		     LeftBack;		//	障碍物BOX的左后点在局部坐标系中的坐标
	fPoint 		     LeftFront;		//	障碍物BOX的左前点在局部坐标系中的坐标
	fPoint 		     RightBack;		//	障碍物BOX的右后点在局部坐标系中的坐标
	fPoint 		     RightFront;	//	障碍物BOX的右前点在局部坐标系中的坐标
	float			 Heading;		//	局部坐标系中的速度朝向,单位是度
	float			 v;				//	局部坐标系中的相对移动速度
	float			 a;				//	局部坐标系中的绝对移动加速度，无法提供则赋0 
	int				 ObsLaneNo;		//  障碍物所在车道信息
	EObstacleClass	 ObstacleID;	//	障碍物分类信息
	union UAddInfo						//  额外的信息：第一位为nTrackID;   其余位置保留； 
	{
		struct 
		{
			int nTrackID;
			int nAge;
			int nTest; 
			int nObsTypeAge;  // 障碍物被观测到具有属性的次数，目前该值用于IFV与LUX融合时使用
			int nObsSource;	  // 1:FLUX 2:RLUX 3:HDL 4:IFV 5:LUTM 6:RUTM 7:OutofRoadEdge
			int nReservedArr[5];        // reserved 
		};
		int data[10];
		union UAddInfo()
		{
			nAge = 0;
			nTrackID = -1;
			nObsTypeAge = 0;
		}
	}AddInfo;

};

// 障碍物结构
// ObstacleMaxNum最大障碍物数量
struct SObstacle
{
	BOOL			IsOK;						//  TRUE – 有 FALSE – 没有
	int 			ObstacleNum;				//	障碍物数量
	SObstacleInfo	Obs[ObstacleMaxNum];		//	障碍物数组
};

//车辆状态结构体
struct AutoVeh
{
	float s;
	float d;
	float theta;
	float v;
	float l_f;
	float l_r;

	AutoVeh()
	{
	    s=0;
		d=0;
		theta=0;
		v=0;
		l_f=4;
		l_r=1;
	}
};
