#pragma once
#include "stdafx.h"
#include "struct_fun.h"
#include "Matrix.h"
#define LayerNum 10;
#define BranchNum 3;
#define dt 0.1;





struct TreeNode
{
	float s;
	float d;
	float theta;
	float v;
	float cost;
	TreeNode()
	{
		s=0;
		d=0;
		theta=0;
		v=0;
		cost=0;
	}
};

struct FrenetLinePt 
{
	float s;
	float d;
	float kappa;
	int type;
	FrenetLinePt()
	{
		s=0;
		d=0;
		kappa=0;
		type=-1;
	}
};
struct FrenetLaneLine
{
	BOOL IsOk;
	int LaneLineNo;
	int ValidNum;
	vector<FrenetLinePt> FrenetLinePT;
	FrenetLaneLine()
	{
		IsOk =FALSE;
		FrenetLinePT.resize(600);
	}
};
struct FrenetObsVeh
{
	float s;
	float d;
	float theta;
	float v;
	float a;
	float len;
	int ObsLaneNo;
	FrenetObsVeh()
	{
		s=0;
		d=0;
		theta=0;
		v=0;
		a=0;
		len=5;
		ObsLaneNo=9;
	}
};
struct FrenetObsHuman
{
	float s;
	float d;
	float theta;
	float v;
	float a;
	FrenetObsHuman()
	{
		s=0;
		d=0;
		theta=0;
		v=0;
		a=0;
	}
};
struct FrenetStaticObs
{
	float s;
	float d;
	float r;
	float sem_len;
	FrenetStaticObs()
	{
		s=0;
		d=0;
		r=0;
		sem_len=0;
	}

};

struct FrenetLaneMark
{
	BOOL IsOk;
	int LaneNo;//当前车体所在绝对车道号，从最右车道为0起，向左依次累加
	double LaneWidth;//车道宽
	FrenetLaneLine		FrenetLineR2;			//车体参考点右侧第二根
	FrenetLaneLine		FrenetLineR1;			//车体参考点右侧第一根
	FrenetLaneLine		FrenetLineL1;			//车体参考点左侧第一根
	FrenetLaneLine		FrenetLineL2;			//车体参考点左侧第二根

	FrenetLaneMark()
	{
	    IsOk=FALSE;
		LaneNo=9;
		LaneWidth=3.75;
	}

};
struct VelocityModel 
{
	float TargetSpeed;
	float a;
	float Speed;
	float Reala;
	VelocityModel()
	{
		TargetSpeed=0;
		a=0;
		Speed=0;
		Reala=0;
	}
};
class TrafficModel
{
public:
	AutoVeh veh;
	vector<vector< TreeNode >> TreeMap;//两层向量容器，依次Map,Layer，Node。
	//vector<float> velocity;//
	FrenetLaneLine   FrenetLane[4];
	FrenetObsVeh     ObsVeh[20];            //障碍车信息
	FrenetObsHuman   ObsHuman[20];         //行人
	FrenetStaticObs  StaticObs[20];       //静态语义障碍物
	int vehNum;
	int humanNum;
	int staticObsNum;
	vector<VelocityModel> VelocityFile;

	TrafficModel(void);
	~TrafficModel(void);


	BOOL GetOptVelocity(SLane &LineInfo,SObstacle &ObsInfo, float t,float a);//获取最优目标加速度及时间
	BOOL GetVelocityFile(float t,float a,vector<float> &velocity);//使用车辆状态估计模型预测车辆速度


    BOOL TrafficSetting(SLane &LineInfo,SObstacle &ObsInfo,FrenetLaneMark &tLaneMark);//车道基本信息状态设定，将笛卡尔转到FreNet下
	BOOL GetFrenetBaseLine(SLaneLine &line, FrenetLaneLine &FrenetLine);
	BOOL GetFrenetLine(SLaneLine &line,SLaneLine &DKBaseLine,FrenetLaneLine &FrenetLine,FrenetLaneLine &FreneBasetLine);
	BOOL GetFrenetObs(SObstacle &Obs,SLaneLine &DKBaseLine,FrenetLaneMark &FrenetLine);


	float SigFunc(float x,float p,float x0);
	float DynamicOccupancy(TreeNode &node,FrenetObsVeh &Obsveh,AutoVeh &veh, vector< vector<float> >refline,int samples);
	float ReactionProb(float t,float t_max,float t_min);
	float StaticOccupancy(TreeNode &node,FrenetStaticObs  StaticObs[],int staticObsNum);
	float SwerveProb(float v,float p_max,float del_theta);
	float LaneMarkCost(TreeNode &node,FrenetLaneMark &LaneMark);//车道线势场计算
	float DynamicLaneRightCost(TreeNode &node,FrenetLaneMark &LaneMark);//动态车道占据势场
	float StaticLaneRightCost(TreeNode &node,FrenetLaneMark &LaneMark);//静态车道占据势场
	float HumanCost(TreeNode &node);//行人占据势场

	float GetStepDis(vector<float> &velocity,int LayerIndex);//获取某一层的步长
	BOOL GetTreeMap(vector<vector< TreeNode >> &TreeMap);//构建状态树
	BOOL BellmanIteration(vector<vector< TreeNode >> &TreeMap);//贝尔曼值迭代
	BOOL OptPolicy(vector<vector< TreeNode >> &TreeMap,vector<TreeNode> &OptNode);//最优路径点搜索

	void Fitting(vector<fPoint> Samples, double* a, int nParam);//曲线拟合
	int ReadVelocityFile(const string &FileName);
	
};


