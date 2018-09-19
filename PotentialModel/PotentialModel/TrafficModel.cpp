#include "StdAfx.h"
#include "TrafficModel.h"
#include<cmath>
#include<vector>
#define pi 3.1415926


TrafficModel::TrafficModel(void)
{
	vehNum=0;
	humanNum=0;
	staticObsNum=0;
}


TrafficModel::~TrafficModel(void)
{




}


  BOOL TrafficModel::TrafficSetting(SLane &LineInfo,SObstacle &ObsInfo,FrenetLaneMark &LaneMark)
  {
	  
	  if (LineInfo.IsOK==FALSE)//判断车道线是否有效
		  return FALSE;
	  memset(&LaneMark,0,sizeof(FrenetLaneMark));

	  LaneMark.LaneWidth=LineInfo.LaneWidth;

	  SLaneLine DKBaseLine;
	

	  if (  LineInfo.LaneNo==3 || LineInfo.LaneNo==2)//挑选最右侧车道线为基准线
	  {
		  memcpy(&DKBaseLine,&LineInfo.LineR2,sizeof(LineInfo.LineR2));
		  if(LineInfo.LineR2.IsOK)
		  {
			  LaneMark.FrenetLineR2.LaneLineNo=1;
			  GetFrenetBaseLine(LineInfo.LineR2,LaneMark.FrenetLineR2);		      
			  FrenetLane[0]=LaneMark.FrenetLineR2;
		  } 
		  if(LineInfo.LineR1.IsOK)
		  {
			  GetFrenetLine(LineInfo.LineR1,DKBaseLine,LaneMark.FrenetLineR1,LaneMark.FrenetLineR2);
			  LaneMark.FrenetLineR1.LaneLineNo=2;
			  FrenetLane[1]=LaneMark.FrenetLineR1;
		  }
		  if (LineInfo.LineL1.IsOK)
		  {
			  GetFrenetLine(LineInfo.LineL1,DKBaseLine,LaneMark.FrenetLineL1,LaneMark.FrenetLineR2);
			  LaneMark.FrenetLineL1.LaneLineNo=3;
			  FrenetLane[2]=LaneMark.FrenetLineL1;
		  }
		  if (LineInfo.LineL2.IsOK)
		  {
			  GetFrenetLine(LineInfo.LineL2,DKBaseLine,LaneMark.FrenetLineL2,LaneMark.FrenetLineR2);
			  LaneMark.FrenetLineL2.LaneLineNo=4;
			  FrenetLane[3]=LaneMark.FrenetLineL2;
		  }
		 
	  }else if (LineInfo.LaneNo==1)
	  {
		  if (LineInfo.LineR1.IsOK)
		  {
			  memcpy(&DKBaseLine,&LineInfo.LineR1,sizeof(LineInfo.LineR1));
			  GetFrenetBaseLine(LineInfo.LineR1,LaneMark.FrenetLineR1);
			  LaneMark.FrenetLineR1.LaneLineNo=1;
			  FrenetLane[0]=LaneMark.FrenetLineR1;
		  }
		  if (LineInfo.LineL1.IsOK)
		  {
			  GetFrenetLine(LineInfo.LineL1,DKBaseLine,LaneMark.FrenetLineL1,LaneMark.FrenetLineR2);
			  LaneMark.FrenetLineL1.LaneLineNo=2;
			 FrenetLane[1]=LaneMark.FrenetLineL1;
		  }
		  if (LineInfo.LineL2.IsOK)
		  {
			  GetFrenetLine(LineInfo.LineL2,DKBaseLine,LaneMark.FrenetLineL2,LaneMark.FrenetLineR2);
			  LaneMark.FrenetLineL2.LaneLineNo=3;
			  FrenetLane[2]=LaneMark.FrenetLineL2;
		  }		
	  }

	  
		  GetFrenetObs(ObsInfo,DKBaseLine,LaneMark);

	 return TRUE;
  }

  BOOL TrafficModel::GetFrenetBaseLine(SLaneLine &line, FrenetLaneLine &FrenetLine)
  {
	  vector<fPoint> samples;//曲线拟合基本采样点
	  FrenetLine.IsOk = TRUE;

	  int LinePointNum = line.ValidNum;
	  FrenetLine.ValidNum=line.ValidNum;
	  double a[4]={0,0,0,0};
	  float s=0;
	  int cecl=(int)LinePointNum/5;
	  for (int i= 0; i<cecl*5; i=i+5)
	  {
		  FrenetLinePt tempPoint;

			for (int j=0;j<5;j++)
			{
				samples.push_back(line.LinePt[i+j]);

			}
			memset(&a, 0, sizeof(double)*4);
			Fitting(samples,a,4);
			samples.clear();

		
			for (int k=0;k<5;k++)
			{
				tempPoint.s = s;
				tempPoint.d = 0;		

				float dy = 3*a[3]*pow(line.LinePt[i+k].x,2) + 2*a[2]*line.LinePt[i+k].x + a[1];
				float d2y = 6*a[3]*line.LinePt[i+k].x + 2*a[2];
				if(i+k==0)
					tempPoint.kappa=0;
				else
					tempPoint.kappa = d2y/pow(1+dy*dy, (float)1.5);
				tempPoint.type = line.LaneLineClass;
				FrenetLine.FrenetLinePT.push_back(tempPoint);
				s += line.LinePt[i+k].Pt2PtDist(line.LinePt[i+k+1]);

				
				memset(&tempPoint,0,sizeof(tempPoint));
		   }
		 
	 }
		  
	  return TRUE;
  }
  BOOL TrafficModel::GetFrenetLine(SLaneLine &line,SLaneLine &DKBaseLine,FrenetLaneLine &FrenetLine,FrenetLaneLine &FrenetBaseLine)
  {
	  int Num = DKBaseLine.ValidNum;
	  int index = 0;
	  
	  float dtemp=1000;
	  FrenetLinePt temp;
	  FrenetLine.IsOk=TRUE;
	  FrenetLine.ValidNum=line.ValidNum;
	  for (int j=0;j<line.ValidNum;j++)
	  {
		  float d = 1000;
		  float s=0;
		  for(int i = 0; i<Num ; i++)
		  {
			  dtemp = line.LinePt[j].Pt2PtDist(DKBaseLine.LinePt[i]);
			  if (dtemp<=d)
			  {
				  d=dtemp;
				  index = i;
			  }else
				  break;
			  if(i!=0)
				  s+=DKBaseLine.LinePt[i-1].Pt2PtDist(DKBaseLine.LinePt[i]);
			  else 
				  continue;
			 
		 }
		  float  temps=0;
		  fPoint A=DKBaseLine.LinePt[index];
		  fPoint B=DKBaseLine.LinePt[index+1];
		  fPoint C=line.LinePt[j];
		  fPoint AB;
			     AB.x=B.x-A.x;
		         AB.y=B.y-A.y;
		  fPoint AC;
			     AC.x=C.x-A.x;
		         AC.y=C.y-A.y;
		  for (int i=0;i<10;i++)
		  {
			  fPoint insertpoint;
			         insertpoint.x=AB.x/10*i+A.x;
			         insertpoint.y=AB.y/10*i+A.y;
			  dtemp = insertpoint.Pt2PtDist(C);
			  if (dtemp<=d)
			  {
				  d=dtemp;
			  }
			  else
			  {
				  temps=sqrt(pow(AB.x*(i-1)/10,2)+pow(AB.y*(i-1)/10,2));
				  break;
			  }
		  }

		
		  temp.s=s+temps;
		  temp.d=d;
		  temp.kappa=(FrenetBaseLine.FrenetLinePT[index].kappa+FrenetBaseLine.FrenetLinePT[index].kappa)/2;
		  temp.type=line.LaneLineClass;
		  FrenetLine.FrenetLinePT.push_back(temp);
		  memset(&temp,0,sizeof(temp));
	  }
	  return TRUE;
  }

  BOOL TrafficModel::GetFrenetObs(SObstacle &Obs,SLaneLine &DKBaseLine,FrenetLaneMark &FrenetLine)
  {  

	  FrenetObsVeh      TempObsVeh;            //障碍车信息
	  FrenetObsHuman    TempObsHuman;         //行人
	  FrenetStaticObs    TempStaticObs;       //静态语义障碍物


	  fPoint A;
	  fPoint B;
	  fPoint C;
	  fPoint AB;
	  fPoint AC;
	  fPoint insertpoint;

	  for (int j=0;j<Obs.ObstacleNum;j++)
	  {
		   float dtemp = 1000;
		  float d = 1000;
		  int BaseLinePtNum=DKBaseLine.ValidNum;
		  int index=0;
		  float s=0;
		  float theta=0;
		  for (int i=0;i<BaseLinePtNum;i++)
		  {
			  dtemp=Obs.Obs[j].Center.Pt2PtDist(DKBaseLine.LinePt[i]);
			  if (dtemp<=d)
			  {
				  d=dtemp;
				  index=i;
			  }
			  else
				  break;
			  if(i!=0)
				  s+=DKBaseLine.LinePt[i-1].Pt2PtDist(DKBaseLine.LinePt[i]);
			  else continue;
			
		  }


		  A=DKBaseLine.LinePt[index];

		  B=DKBaseLine.LinePt[index+1];
		
		  C=Obs.Obs[j].Center;
		  
		  AB.x=B.x-A.x;
		  AB.y=B.y-A.y;
		  
		  AC.x=C.x-A.x;
		  AC.y=C.y-A.y;
		  float temps=0;
		  for (int i=0;i<10;i++)
		  {
		
			  insertpoint.x=AB.x/10*i+A.x;
			  insertpoint.y=AB.y/10*i+A.y;
			  dtemp = insertpoint.Pt2PtDist(C);
			  if (dtemp<=d)
			  {
				  d=dtemp;
			  }
			  else
			  {
				  temps=sqrt(pow(AB.x*(i-1)/10,2)+pow(AB.y*(i-1)/10,2));
				  break;
			  }
		  }
		  s=s+temps;

		  if (Obs.Obs[j].ObstacleID==Vehicle)
		  {
			  TempObsVeh.s=s;
			  TempObsVeh.d=d;
			  TempObsVeh.v=Obs.Obs[j].v;
			  TempObsVeh.a=0;
			  TempObsVeh.len=5;
			  if (FrenetLine.FrenetLineR2.LaneLineNo==1)
			  {
				  TempObsVeh.theta=atan((1-FrenetLine.FrenetLineR2.FrenetLinePT[index].kappa*Obs.Obs[j].v)*tan(Obs.Obs[j].Heading));
			  }else
			  {
				  TempObsVeh.theta=atan((1-FrenetLine.FrenetLineR1.FrenetLinePT[index].kappa*Obs.Obs[j].v)*tan(Obs.Obs[j].Heading));
			  }
			  ObsVeh[vehNum]=TempObsVeh;
			  vehNum++;
		  }else if (Obs.Obs[j].ObstacleID == Human)
		  {
			  TempObsHuman.s=s;
			  TempObsHuman.d=d;
			  if (FrenetLine.FrenetLineR2.LaneLineNo==1)
			  {
				  TempObsVeh.theta=atan((1-FrenetLine.FrenetLineR2.FrenetLinePT[index].kappa*Obs.Obs[j].v)*tan(Obs.Obs[j].Heading));
			  }else
			  {
				  TempObsVeh.theta=atan((1-FrenetLine.FrenetLineR1.FrenetLinePT[index].kappa*Obs.Obs[j].v)*tan(Obs.Obs[j].Heading));
			  }
			  TempObsHuman.v=Obs.Obs[j].v;
			  TempObsHuman.a=0;
			  ObsHuman[humanNum]=TempObsHuman;
			  humanNum++;
		  }else
		  {
			  TempStaticObs.s=s;
			  TempStaticObs.d=d;
			  TempStaticObs.r=1.5;
			  TempStaticObs.sem_len=abs(Obs.Obs[j].LeftFront.Pt2PtDist(Obs.Obs[j].LeftBack));
			  StaticObs[staticObsNum]=TempStaticObs;
			  staticObsNum++;
		  }
	  }

	  
	  return TRUE;
  }


  BOOL TrafficModel::GetOptVelocity(SLane &LineInfo,SObstacle &ObsInfo, float t,float a)
  {




	  return TRUE;
  }
  BOOL TrafficModel::GetVelocityFile(float t,float a,vector<float> &velocity)
  {
	  return TRUE;
  }

 float TrafficModel::LaneMarkCost(TreeNode &node,FrenetLaneMark &LaneMark)
  {
	  float left_d=0,right_d=0,left_A=0,right_A=0;
	  int idx=0,point_Laneidx=0;
	  int samples=100;  //车道线采样点个数一定，为100，但点与点之间的间距不确定
	  point_Laneidx=ceil(node.d/LaneMark.LaneWidth);
	  if(node.d==0)point_Laneidx=1;

	  for(int i=0;i<samples;i++)
	  {
		  if (node.s<FrenetLane[point_Laneidx].FrenetLinePT[i].s)
		 {
		     idx=i;
			 break;
		 }
	  }

	  left_d= FrenetLane[point_Laneidx].FrenetLinePT[idx].d-node.d;
	  right_d=node.d- FrenetLane[point_Laneidx-1].FrenetLinePT[idx].d;

	  if( FrenetLane[point_Laneidx].FrenetLinePT[idx].type==0)
		  left_A=50;
	  else
		  { 
			if( FrenetLane[point_Laneidx].FrenetLinePT[idx].type==1)
			    left_A=75;
		    else
			    left_A=95;
	      }

	  if( FrenetLane[point_Laneidx-1].FrenetLinePT[idx].type==0)
		  right_A=50;
	  else
		  { 
			if( FrenetLane[point_Laneidx-1].FrenetLinePT[idx].type==1)
			    right_A=75;
		    else
			    right_A=95;
	      }

	  float q=-1;
	  float cost_left,cost_right,lanemark_cost;
	  float width=LaneMark.LaneWidth;
	  cost_left=SigFunc(left_d,q,width)*left_A;
	  cost_right=SigFunc(right_d,q,width)*right_A;

	  if(left_d==0) cost_right=0;
	  if(right_d==0) cost_left=0;

	  lanemark_cost=cost_left+cost_right;  //最大值75

	  return lanemark_cost;
  }


  float TrafficModel::SigFunc(float x,float p,float x0)
  {
      float y=x;
	  float y0=(1-exp(p/x0))/(1+exp(p/x0));
	  y=(1-exp(p/x))/(1+exp(p/x))-x*y0/x0;
      return y;
  }


  float TrafficModel::DynamicLaneRightCost(TreeNode &node,FrenetLaneMark &LaneMark)
  {
	  int samples=600;  //车道线采样点个数一定，比赛期间为100个，但点与点之间的间距不确定
	  int point_Laneidx=ceil(node.d/LaneMark.LaneWidth);  //点所在车道号
	  if(node.d==0)point_Laneidx=1;
	  vector< vector<float> > refline(samples);
	  
	  for(int i=0;i<samples;i++)   //refline中存储的为点所在车道中线信息
	  {
		  refline[i].push_back(FrenetLane[point_Laneidx-1].FrenetLinePT[i].s);
		  refline[i].push_back((FrenetLane[point_Laneidx-1].FrenetLinePT[i].d+FrenetLane[point_Laneidx].FrenetLinePT[i].d)/2);
		  refline[i].push_back(FrenetLane[point_Laneidx-1].FrenetLinePT[i].kappa);
	  }
	  
	  float p=0,dynamiccost=0,dynamic_cost=0;

	  for(int i=0;i<vehNum;i++) 
	  {
		 ObsVeh[i].ObsLaneNo=ceil(ObsVeh[i].d/LaneMark.LaneWidth);
		 if(point_Laneidx==ObsVeh[i].ObsLaneNo)   //如果障碍车辆和点在一条车道上则计算障碍车辆对于该点的占用率
		 {
			 p=DynamicOccupancy(node,ObsVeh[i],veh,refline,samples);   //一次计算一个障碍车辆在该点的占用率
			 if(dynamiccost<p)dynamiccost=p;    //取最大作为最终占用率
		 }
	  }
		  
	  dynamic_cost=(dynamiccost*95)>100?100:(dynamiccost*95);

	  return dynamic_cost;
  }


  float TrafficModel::DynamicOccupancy(TreeNode &node,FrenetObsVeh &Obsveh,AutoVeh &veh,vector< vector<float> >refline,int samples)
  {
      float safedis=4;
	  float p=0,t=0;
	  float t_obsmax=2.2;  //最大反应时间
	  float t_obsmin=1.2;
	  float t_obsmid=(t_obsmax+t_obsmin)/2;
	  float t_vehmax=1.1;
	  float t_vehmin=0.3;
	  float t_vehmid=(t_vehmax+t_vehmin)/2;

	  float a_min=-4;  //最大减速度
	  if ((node.s>=(Obsveh.s-Obsveh.len/2-veh.l_f-safedis))&&(node.s<=(Obsveh.s+Obsveh.len/2+veh.l_r+safedis))) p=1.1;
	  else
	  {	
		 if (node.s>(Obsveh.s+Obsveh.len/2+veh.l_r))   //点在障碍车前
		 {
			float c=node.s-Obsveh.s+(pow(Obsveh.v,2)-pow(veh.v,2))/(2*a_min)-Obsveh.len/2-veh.l_r;
		  
			for(int i=0;i<(samples-1);i++)
			{
				if((refline[i][0]>=Obsveh.s)&&(refline[i][1]<node.s))
					c=c-refline[i][2]*refline[i][1]*(refline[i+1][0]-refline[i][0]);  //将S轴上距离转换为车道中线距离
				else
					if(refline[i][0]>=node.s)break;

			}

			t=c/Obsveh.v;
			if(t<t_obsmin)p=1;  //t小于最短反应时间，一定碰撞
			if((t>=t_obsmin)&&(t<t_obsmid))p=1-ReactionProb(t,t_obsmax,t_obsmin)*(t-t_obsmin)/2;  //ReactionProb得到t对应的概率密度函数上的值进而求得碰撞概率
			if((t>=t_obsmid)&&(t<=t_obsmax))p=ReactionProb(t,t_obsmax,t_obsmin)*(t_obsmax-t)/2;
			if(t>t_obsmax)p=0;  //当t大于最长反应时间时一定不碰撞，碰撞概率p为0

		 }

		 else  //障碍车在前
		 {
			float c=Obsveh.s-node.s+(pow(veh.v,2)-pow(Obsveh.v,2))/(2*a_min)-Obsveh.len/2-veh.l_f-safedis;
			for(int i=0;i<(samples-1);i++)
			{
				if((refline[i][0]>=node.s)&&(refline[i][0]<Obsveh.s))
					c=c-refline[i][2]*refline[i][1]*(refline[i+1][0]-refline[i][0]);  //将S轴上距离转换为车道中线距离
				else
					if(refline[i][0]>=Obsveh.s)break;

			}
			t=c/veh.v;
			if(t<t_vehmin)p=1;
			if((t>=t_vehmin)&&(t<t_vehmid))p=1-ReactionProb(t,t_obsmax,t_obsmin)*(t-t_obsmin)/2;   //根据概率密度函数计算碰撞概率
			if((t>=t_vehmid)&&(t<=t_vehmax))p=ReactionProb(t,t_obsmax,t_obsmin)*(t_obsmax-t)/2;
			if(t>t_vehmax)p=0;
		  
		 }
	  }

      return p;
  }

  float TrafficModel::ReactionProb(float t,float t_max,float t_min)   //根据概率密度函数计算碰撞概率
  {
      float t_mid=(t_max+t_min)/2;
	  float p_max=2/(t_max-t_min);
	  float prob;

	  if((t>=t_min)&&(t<t_mid))prob=p_max*(t-t_min)/(t_mid-t_min);
	  else
	      if((t>=t_mid)&&(t<=t_max))prob=-p_max*(t-t_max)/(t_max-t_mid);
		  else prob=0;

	  return prob;
  }


  float TrafficModel::StaticLaneRightCost(TreeNode &node,FrenetLaneMark &LaneMark)
  {
	  int LaneNum,Staticobs_idx;  //Staticobs_idx为障碍车辆所在车道号

	  if(LaneMark.FrenetLineR2.LaneLineNo==1)LaneNum=3;
	  else LaneNum=2;

	  vector<int>index(LaneNum,0);   //存放各车道障碍车辆数目,初始化为0
	  vector<FrenetStaticObs>Staticobs1;
	  vector<FrenetStaticObs>Staticobs2;
	  vector<FrenetStaticObs>Staticobs3;
	  vector<FrenetStaticObs>LaneObs;

	  int staticObsNum=staticObsNum;  //静态语义障碍物的数目

	  for(int i=0;i<staticObsNum;i++)   
	  {
		  Staticobs_idx=ceil(StaticObs[i].d/LaneMark.LaneWidth);  //障碍车辆所在车道号
		  index[Staticobs_idx]=index[Staticobs_idx]+1;  //各车道障碍车辆数目
		  if(Staticobs_idx==1)Staticobs1.push_back(StaticObs[i]);
		  if(Staticobs_idx==2)Staticobs2.push_back(StaticObs[i]);
		  if((Staticobs_idx==3)&&(LaneNum==3))Staticobs3.push_back(StaticObs[i]);
	  
	  }

	  float Smin=0,Smax=0;  //所有障碍物所在范围的起点和终点
	  int lmin=0,lmax=0;    //所有障碍物所在范围的起点和终点所对应的索引值
	  for(int i=0;i<staticObsNum;i++)
	  {
		  if(Smin>StaticObs[i].s)Smin=StaticObs[i].s,lmin=i;
		  if(Smax<StaticObs[i].s)Smax=StaticObs[i].s,lmax=i;
	  }
	  Smin=Smin-StaticObs[lmin].r-veh.l_f;
	  Smax=Smax+StaticObs[lmin].r+veh.l_r+0.5;


	  vector< vector<float> >S_flag;   //存储每条车道线上障碍物范围的起点和终点
	  vector<float>a(LaneNum);
	  vector<float>b(2);
	  S_flag.push_back(a);
	  S_flag.push_back(b);

	  for(int i=1;i<=LaneNum;i++)
	  {
		  if((i==1)&&(Staticobs1.size()>0))LaneObs=Staticobs1;
		  if((i==2)&&(Staticobs2.size()>0))LaneObs=Staticobs2;
		  if((i==3)&&(Staticobs3.size()>0))LaneObs=Staticobs3;
		  if(LaneObs.size()==0)continue;
		  
		  S_flag[i][1]=0;  //障碍物范围起点
		  S_flag[i][2]=0;  //障碍物范围终点
		  int LaneObs_Num=LaneObs.size();

		  for(int j=0;j<LaneObs_Num;j++)
		  {
			  if((LaneObs[j].sem_len>0)&&((LaneObs[j].s+LaneObs[j].sem_len)>0))
			  {
			      if((S_flag[i][1]==0)&&(S_flag[i][2]==0))
				  {
				      S_flag[i][1]=LaneObs[j].s;
					  S_flag[i][2]=LaneObs[j].s+LaneObs[j].sem_len;
				  }
				  else
				  {
				      if(S_flag[i][1]>LaneObs[j].s)S_flag[i][1]=LaneObs[j].s;
				      if(S_flag[i][2]<LaneObs[j].s+LaneObs[j].sem_len)S_flag[i][2]=LaneObs[j].s+LaneObs[j].sem_len;
				  }
			  
			  }
		  
		  }

	  }

	  int point_Laneidx=ceil(node.d/LaneMark.LaneWidth);  //点所在车道号
	  if(node.d==0)point_Laneidx=1;
	  float p,Static_cost;

	  if((node.s>S_flag[point_Laneidx][1])&&(node.s<S_flag[point_Laneidx][2])) Static_cost=100;
	  else
	  {
	      if((node.s>Smin)&&(node.s<Smax))
		  {
		      p=StaticOccupancy(node,StaticObs,staticObsNum);
			  Static_cost=100*p;

		  }
		  else Static_cost=0;
	  }
	  
	  return Static_cost;
  }


  float TrafficModel::StaticOccupancy(TreeNode &node,FrenetStaticObs  StaticObs[],int staticObsNum)
  {
      float w_auto=2;
	  float l_auto=3.8;
	  float MinDis=10;
	  float dis,p,q=-1;

	  for(int i=0;i<staticObsNum;i++)
	  {
		  dis=sqrt(pow((node.s-StaticObs[i].s),2)+pow((node.d-StaticObs[i].d),2));
		  if(dis<=MinDis)MinDis=dis;

	  }

	  if(MinDis<w_auto/2)p=1;
	  else
		  if(MinDis>l_auto)p=0;
		  else
			  p=SigFunc((MinDis-w_auto/2),q,l_auto);

      return p;
  }


  float TrafficModel::HumanCost(TreeNode &node)
  {
	  float t_auto=0.7;
	  float t_hum=1.7;
	  float Hum_cost=0;
	  float del_theta,dis,p,p_max,v,vp,c;
	  int Hum_Num=humanNum;

	  for(int i=0;i<Hum_Num;i++)
	  {
		  del_theta=atan((node.d-ObsHuman[i].d)/(node.s-ObsHuman[i].s))-ObsHuman[i].theta;
		  dis=sqrt(pow((node.s-ObsHuman[i].s),2)+pow((node.d-ObsHuman[i].d),2));
		  v=ObsHuman[i].v;
		  p_max = (v/3+0.5)/pi;
		  p=SwerveProb(v,p_max,del_theta);
		  

		  if(v<1.5)v=1.5;
		  vp=v*pow((p/p_max),2)/2;

		  if(dis<=t_auto*vp) c=100;
		  else
		  {
		      if(dis>t_hum*vp)c=0;
			  else
				   c=-100*(dis-t_hum*vp)/(t_hum*vp-t_auto*vp);
		  }

		  Hum_cost=Hum_cost+c;
	  
	  }
  
	  return Hum_cost;
  }


  float TrafficModel::SwerveProb(float v,float p_max,float del_theta)  //返回行人突然转向的概率
  {
      float p;
	  while(del_theta>pi)  //将del_theta转换到[-pi,pi]
	  {
	      del_theta = del_theta-2*pi;
	  }
	  while(del_theta<-pi)
	  {
	      del_theta = del_theta+2*pi;
	  }

	  if(v<=1.5)
	  {
	      if(del_theta<=0) p=del_theta*(2*pi*p_max-1)/pow(pi,2)+p_max;
		  else
			  p=del_theta*(1-2*pi*p_max)/pow(pi,2)+p_max;
	  }
	  else
	  {
	      if((del_theta>(-1/p_max))&&(del_theta<=0)) p=pow(p_max,2)*del_theta+p_max;
		  else
		  {
		      if((del_theta>0)&&(del_theta<=1/p_max))p=-pow(p_max,2)*del_theta+p_max;
			  else
				  p=0;
		  }
			  
	  }

      return p;
  }


  float TrafficModel::GetStepDis(vector<float> &velocity,int LayerIndex)
  {
	  return 0;
  }
  BOOL TrafficModel::GetTreeMap(vector<vector< TreeNode >> &TreeMap)
  {
	  return TRUE;
  }
  BOOL TrafficModel::BellmanIteration(vector<vector< TreeNode >> &TreeMap)
  {
	  return TRUE;
  }
  BOOL TrafficModel::OptPolicy(vector<vector< TreeNode >> &TreeMap,vector<TreeNode> &OptNode)
  {
	  return TRUE;
  }

  // 曲线拟合
  void TrafficModel::Fitting(vector<fPoint> Samples, double* a, int nParam)
  {
	  int i, j, m, n;
	  m = Samples.size();
	  double Cond = 0;
	  n = nParam;
	  double A[N_PARAM][N_PARAM];
	  double b[N_PARAM];
	  for (i = 0; i < n; i ++) {
		  for (j = 0; j < n; j ++) {
			  A[i][j] = 0.;
		  }
		  b[i] = 0.;
	  }
	  A[0][0] = m;
	  for (i = 0; i < m; i ++) {
		  b[0] += Samples[i].y;
	  }
	  for (j = 1; j < n; j ++) {
		  for (i = 0; i < m; i ++) {
			  A[j][0] += pow(double(Samples[i].x), j);
			  A[n - 1][j] += pow(double(Samples[i].x), n + j - 1);
			  b[j] += pow(double(Samples[i].x), j) * Samples[i].y;
		  }
	  }
	  for (j = 1; j < n - 1; j ++) {
		  for (i = j; i < n - 1; i ++) {
			  A[i][j] = A[i + 1][j - 1];
		  }
	  }
	  for (i = 0; i < n - 1; i ++) {
		  for (j = i + 1; j < n; j ++) {
			  A[i][j] = A[j][i];
		  }
	  }
	  Matrix matrix(n);
	  matrix.m_SetMatrix(A);
	  Cond = matrix.m_TriangDecomp();
	  if (Cond < 0.) 
	  {
		  return;
	  }
	  matrix.m_SetVector(b);
	  matrix.m_BackSubstitute();
	  matrix.m_GetVector(b);
	  for (i = 0; i < n; i ++) {
		  a[i] = b[i];
	  }
	  return;
  }
  //---------------------------------------------------------------------------------------
  // m_vKeyTraQuat m_MapTra_ptr 读取制作地图的轨迹和关键帧变换信息
  //---------------------------------------------------------------------------------------
int ReadVelocityFile(const string &FileName)
{
	ifstream ifData;
	char TraName[200] = "\0";
	sprintf_s(TraName,"%sRst.txt",FileName.c_str());
	ifData.open(TraName);
	if (!ifData)
	{
		cout<<"Cannot open the file!"<<endl;
		exit(-1);
	}
	//stringstream ss;
	string tmpStr;

	VelocityModel temp;


}
  //{

	 // std::stringstream ss;
	 // std::string tmpStr;

	 // sTraQuat tmpTraQuat;
	 // pcl::PointXYZ tmpTraData;
	 // pcl::PointCloud<pcl::PointXYZ> TraData;
	 // int nId = 0;
	 // while (!ifData.eof())
	 // {
		//  getline(ifData, tmpStr);
		//  if (tmpStr.empty())
		//  {
		//	  continue;
		//  }
		//  ss.clear();
		//  ss.str(tmpStr);
		//  ss >> tmpTraQuat.Idx >> tmpTraQuat.tx >> tmpTraQuat.ty >> tmpTraQuat.tz
		//	  >> tmpTraQuat.qx >> tmpTraQuat.qy >> tmpTraQuat.qz >> tmpTraQuat.qw;
		//  vTraAndQuat.push_back(tmpTraQuat);

		//  tmpTraData.x = tmpTraQuat.tx;
		//  tmpTraData.y = tmpTraQuat.ty;
		//  tmpTraData.z = tmpTraQuat.tz;
		//  TraData.push_back(tmpTraData);
	 // }
	 // MapTra_ptr->swap(TraData);
	 // ifData.close();
	 // return 0;
  //}
