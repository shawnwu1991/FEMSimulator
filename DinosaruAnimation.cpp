#include "stdafx.h"
#include "DinosaruAnimation.h"

#pragma comment (lib, "QPSolver.lib")

#include <iomanip>
#include <numeric>
#include <algorithm>
#include <functional>
#include <fstream>
#include <string>

namespace Libin
{

	DinosaruAnimation::DinosaruAnimation()
	{
		curTime = 0.0;
	}

	DinosaruAnimation::~DinosaruAnimation(void)
	{
	}

	bool DinosaruAnimation::LoadWeight(const QString &pathdir)
	{
		// load node coord
		tifstream ifStr;
		ifStr.open((pathdir+ "Model/weightVert.txt").utf16());	

		tstring line;
		while (std::getline(ifStr, line))
		{
			tistringstream sstr(line);
			double tmpValue;
			for(int j=0; j<7; j++)
			{
				sstr >> tmpValue;
				vertWeight.push_back(tmpValue);
			}
		}

		ifStr.close();
		return true;
	}


	bool DinosaruAnimation::LoadCtrlPoint(const QString &pathdir)
	{
		// load node coord
		tifstream ifStr;
		ifStr.open((pathdir+ "Model/ctrlNode.txt").utf16());	
		Vector3d tmpCoord;
		while (ifStr >> tmpCoord.X() >> tmpCoord.Y() >> tmpCoord.Z())
			ctrlPoint.push_back(tmpCoord);
			
		ifStr.close();
		return true;
	}

	bool DinosaruAnimation::LoadRefShape(const QString &pathdir)
	{
		// load ref shape
		tifstream ifStr;
		ifStr.open((pathdir+ "Model/dinosaur_scale.vert").utf16());	
		Vector3d tmpCoord;
		int idex;
		while (ifStr >> idex >>  tmpCoord.X() >> tmpCoord.Y() >> tmpCoord.Z())
		{
			refShape.push_back(tmpCoord);
			//std::cout << "body coord: " << footMode.size() << " " << footMode[footMode.size()-1].X() << " " <<  footMode[footMode.size()-1].Y() 
			//	<< " " <<  footMode[footMode.size()-1].Z() << " " <<  std::endl;
		}
		ifStr.close();
		return true;
	}

	bool DinosaruAnimation::LoadAnimData(const QString &pathdir)
	{
		// load node coord
		tifstream ifStr;
		ifStr.open((pathdir+ "Model/walk.txt").utf16());	

		tstring line;
		while (std::getline(ifStr, line))
		{
			tistringstream sstr(line);
			double tmpValue;
			for(int j=0; j<14; j++)
			{
				sstr >> tmpValue;
				motionData.push_back(tmpValue);
			}
		}

		ifStr.close();
		return true;
	}

	void DinosaruAnimation::Anim_Vert(double t, std::vector<Vector3d>& pos, std::vector<Vector3d>& posLast, std::vector<Vector3d>& vel, std::vector<Vector3d>& force)
	{
		static double angle =0.0;
		static double displace = 0.0;
		static double displace2 = 0.0;
		static double displacey = 0.0;

		static double angle1 = 0.0;
		static double angle2 = 0.0;

		angle += 1*t;
		std::cout << "animation " << cos(angle) << std::endl;
		static bool first = true;
		if(cos(2*angle)>0.0)
		{
			displace += 0.12*cos(2*angle)*t;	
			angle1 = -0.6*cos(2*angle)-0.1;
			angle2 = 0.1*cos(2*angle);
		}
		if(cos(2*angle)<0.0)
		{
			displace2 += -0.12*cos(2*angle)*t;
			angle2 = 0.6*cos(2*angle)-0.1;
			angle1 = -0.1*cos(2*angle);
			first = false;
		}

		displacey += 0.02*cos(4*angle)*t;

		Matrix3d rotMat;
		Matrix3d rotMat2;
		rotMat.MakeEulerXYZ(angle1, 0.0, 0.0);
		rotMat2.MakeEulerXYZ(angle2, 0.0, 0.0);
		
		for(int i=0; i<pos.size(); i++)
		{
			double w1 = vertWeight[i*7+0];
			double w2 = vertWeight[i*7+1];
			double w3 = vertWeight[i*7+2];
			double w4 = vertWeight[i*7+3];
			double w5 = vertWeight[i*7+4];
			double w6 = vertWeight[i*7+5];
			double w7 = vertWeight[i*7+6];

			if(w4>0.93 || w5>0.93 || w6>0.93 || w7>0.93 )
			{
				double sum = w4 + w5 + w6 + w7;
				pos[i] = /*w1*((refShape[i] - ctrlPoint[0])+ctrlPoint[0]) + 
					w2*((refShape[i] - ctrlPoint[1])+ctrlPoint[1])+ 
					w3*((refShape[i] - ctrlPoint[2])+ctrlPoint[2])+*/
					w4/sum*((refShape[i] - ctrlPoint[3])*rotMat+ctrlPoint[3] + Vector3d(0, displacey, displace)) + 
					w5/sum*((refShape[i] - ctrlPoint[4])*rotMat2+ctrlPoint[4] + Vector3d(0, displacey, displace2))+ 
					w6/sum*((refShape[i] - ctrlPoint[5])*rotMat2+ctrlPoint[5] + Vector3d(0, displacey, displace2)) + 
					w7/sum*((refShape[i] - ctrlPoint[6])*rotMat+ctrlPoint[6] + Vector3d(0, displacey, displace));

			}
		}
	}


	void DinosaruAnimation::Anim_Vert_FromData(double t, std::vector<Vector3d>& pos, std::vector<Vector3d>& posLast, std::vector<Vector3d>& vel, std::vector<Vector3d>& force)
	{
		static int idx = 0;

		Matrix3d rotMat;
		Matrix3d rotMat2;
		
		rotMat.MakeEulerXYZ(-3.14*leftKneeAngle[idx]/180.0, 0.0, 0.0);
		rotMat2.MakeEulerXYZ(-3.14*rightKneeAngle[idx]/180.0, 0.0, 0.0);
		
		for(int i=0; i<pos.size(); i++)
		{
			double w1 = vertWeight[i*7+0];
			double w2 = vertWeight[i*7+1];
			double w3 = vertWeight[i*7+2];
			double w4 = vertWeight[i*7+3];
			double w5 = vertWeight[i*7+4];
			double w6 = vertWeight[i*7+5];
			double w7 = vertWeight[i*7+6];

			if(w4>0.93 || w5>0.93 || w6>0.93 || w7>0.93 )
			{
			/*	pos[i] = w4*((refShape[i] - ctrlPoint[3])*rotMat   + LF_dis + leftKnee[idx] ) + 
						  w5*((refShape[i] - ctrlPoint[4])*rotMat2 + RF_dis + rightKnee[idx] )+ 
						  w6*((refShape[i] - ctrlPoint[5])*rotMat2 + LB_dis + leftKnee[idx] ) + 
						  w7*((refShape[i] - ctrlPoint[6])*rotMat  + RB_dis + rightKnee[idx]  );*/
			pos[i] = w4*((refShape[i] - ctrlPoint[3])*rotMat   + LF_dis + leftKnee[idx] ) + 
						  w5*((refShape[i] - ctrlPoint[4])*rotMat2 + RF_dis + rightKnee[idx] )+ 
						  w6*((refShape[i] - ctrlPoint[5])*rotMat2 + LB_dis + leftKnee[idx] ) + 
						  w7*((refShape[i] - ctrlPoint[6])*rotMat  + RB_dis + rightKnee[idx]  );

			}
		}

		idx++;
	}

	void DinosaruAnimation::Process_Data()
	{
		int numFrame = motionData.size()/14;
		for(int i=0; i<numFrame; i++)
		{
			Vector3d left = Vector3d(motionData[i*14+2]/16.7, motionData[i*14+1]/16.7, motionData[i*14+0]/5.7);
			leftKnee.push_back(left);
			leftKneeAngle.push_back(motionData[i*14+3]);
			Vector3d right = Vector3d(motionData[i*14+6]/16.7, motionData[i*14+5]/16.7, motionData[i*14+4]/5.7);
			rightKnee.push_back(right);
			rightKneeAngle.push_back(motionData[i*14+7]);
			Vector3d pel = Vector3d(motionData[i*14+10]/16.7, motionData[i*14+9]/16.7, motionData[i*14+8]/5.7);
			pelvi.push_back(pel);
		}

		LF_dis = ctrlPoint[3] - leftKnee[0];
		RF_dis = ctrlPoint[4] - rightKnee[0];
		LB_dis = ctrlPoint[5] - leftKnee[0];
		RB_dis = ctrlPoint[6] - rightKnee[0];
	}


}		