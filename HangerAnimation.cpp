#include "stdafx.h"
#include "HangerAnimation.h"

#pragma comment (lib, "QPSolver.lib")

#include <iomanip>
#include <numeric>
#include <algorithm>
#include <functional>
#include <fstream>
#include <string>

namespace Libin
{

	HangerAnimation::HangerAnimation()
	{
		curTime = 0.0;
	}

	HangerAnimation::~HangerAnimation(void)
	{
	}

	bool HangerAnimation::LoadWeight(const QString &pathdir)
	{
		// load node coord
		tifstream ifStr;
		ifStr.open((pathdir+ "Model/weightVert.txt").utf16());	

		tstring line;
		while (std::getline(ifStr, line))
		{
			tistringstream sstr(line);
			double tmpValue;
			for(int j=0; j<4; j++)
			{
				sstr >> tmpValue;
				vertWeight.push_back(tmpValue);
			}
		}

		ifStr.close();
		return true;
	}


	bool HangerAnimation::LoadCtrlPoint(const QString &pathdir)
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

	bool HangerAnimation::LoadRefShape(const QString &pathdir)
	{
		// load ref shape
		tifstream ifStr;
		ifStr.open((pathdir+ "Model/hanger_scale_stand.vert").utf16());	
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

	bool HangerAnimation::LoadAnimData(const QString &pathdir)
	{
		// load node coord
		tifstream ifStr;
		ifStr.open((pathdir+ "Model/traj.txt").utf16());	
		Vector3d tmpCoord;
		while (ifStr >> tmpCoord.X() >> tmpCoord.Y() >> tmpCoord.Z())
		{
			motionData.push_back(tmpCoord);
		}

		ifStr.close();

		ifStr.open((pathdir+ "Model/trajOrientation.txt").utf16());	
		while (ifStr >> tmpCoord.X() >> tmpCoord.Y() >> tmpCoord.Z())
			orientData.push_back(tmpCoord);

		ifStr.close();
		return true;
	}

	void HangerAnimation::Anim_Vert(double t, std::vector<Vector3d>& pos, std::vector<Vector3d>& posLast, std::vector<Vector3d>& vel, std::vector<Vector3d>& force)
	{
		static double angle =0.0;
		static double displace = 0.0;
		static double displace2 = 0.0;
		static double displacey = 0.0;

		static double angle1 = 0.0;
		static double angle2 = 0.0;

		angle += 4*t;
		std::cout << "animation " << cos(angle) << std::endl;
		static bool first = true;
	
		angle1 = 0.8*cos(2*angle)/*-0.1*/;
		angle2 = -0.8*cos(2*angle)/*-0.1*/;
		
		if(cos(2*angle)>0.0)
		{
		//angle1 = 0.8*cos(2*angle)/*-0.1*/;
		displace += 0.155*cos(2*angle)*t;
		}
		else
		{
		//angle2 = -0.8*cos(2*angle)/*-0.1*/;
		
		displace2 += -0.155*cos(2*angle)*t;
		}

		displacey += 0.03*cos(4*angle)*t;
		//displacey = 0.1*cos(4*angle)*t;
		displacey = 0.01*cos(4*angle);

		Matrix3d rotMat;
		Matrix3d rotMat2;
		//rotMat.MakeEulerXYZ(/*2*angle1*/0.0, 0.0, /*0.6*angle1*/1.0);
		//rotMat2.MakeEulerXYZ(/*2*angle1*/0.0, 0.0, /*0.6*angle1*/-1.0);
		rotMat.MakeEulerXYZ(-0.5*angle1, 0.0, 0.6*angle1);
		rotMat2.MakeEulerXYZ(0.5*angle1, 0.0, 0.6*angle1);

		Matrix3d rotMat3;
		Matrix3d rotMat4;
		rotMat3.MakeEulerXYZ(0.0,  0.25*angle1, 0.0);
		rotMat4.MakeEulerXYZ(angle1, 0.0, 0.2*angle1);
		
		for(int i=0; i<pos.size(); i++)
		{
			double w1 = vertWeight[i*4+0];
			double w2 = vertWeight[i*4+1];
			double w3 = vertWeight[i*4+2];
			double w4 = vertWeight[i*4+3];

			if(w3>0.9|| w2>0.9)
			{
				double sum = w3 + w2;
				pos[i] = /*w1*((refShape[i] - ctrlPoint[0])+ctrlPoint[0]) + 
					w2*((refShape[i] - ctrlPoint[1])+ctrlPoint[1])+ 
					w3*((refShape[i] - ctrlPoint[2])+ctrlPoint[2])+*/
					w3/sum*((refShape[i] - ctrlPoint[2])*rotMat+ctrlPoint[2]  + Vector3d(0,  0.2*displacey,  displace)) + 
					w2/sum*((refShape[i] - ctrlPoint[1])*rotMat2+ctrlPoint[1] + Vector3d(0,  0.2*displacey,  displace2));

				//pos[i] = pos[i] - Vector3d(0.0, -1.0, 0.0);
			}

			if(w1>0.8)
			{
				double sum = w1;
				pos[i] = /*w1*((refShape[i] - ctrlPoint[0])+ctrlPoint[0]) + 
					w2*((refShape[i] - ctrlPoint[1])+ctrlPoint[1])+ 
					w3*((refShape[i] - ctrlPoint[2])+ctrlPoint[2])+*/
					w1/sum*((refShape[i] - ctrlPoint[0])*rotMat3+ctrlPoint[0] + Vector3d(0, displacey, 0.5*(displace+displace2)));
			}

			//if(w5>0.98 || w6>0.98)
			//{
			//	double sum = w5 + w6;
			//	pos[i] = /*w1*((refShape[i] - ctrlPoint[0])+ctrlPoint[0]) + 
			//		w2*((refShape[i] - ctrlPoint[1])+ctrlPoint[1])+ 
			//		w3*((refShape[i] - ctrlPoint[2])+ctrlPoint[2])+*/
			//		w5/sum*((refShape[i] - ctrlPoint[4])+ctrlPoint[4] + Vector3d(0, displacey, 0.5*(displace+displace2))) + 
			//		w6/sum*((refShape[i] - ctrlPoint[5])+ctrlPoint[5] + Vector3d(0, displacey, 0.5*(displace+displace2)));
			//}

			//static double accu_time = 0.0;
		 //   accu_time += t;
			//double angle3 = 1.5 * sin(accu_time*10);

			////std::cout << "angle3: " << angle3 * 180.0/3.14 << std::endl;
		
			//Matrix3d rotMat3;
		 //   rotMat3.MakeEulerXYZ(0.9, 0.0, 0.0);
	
			//if(w1>0.99)
			//{
			//	double sum = w2 ;
			//	pos[i] = /*w1*((refShape[i] - ctrlPoint[0])+ctrlPoint[0]) + 
			//		w2*((refShape[i] - ctrlPoint[1])+ctrlPoint[1])+ 
			//		w3*((refShape[i] - ctrlPoint[2])+ctrlPoint[2])+*/
			//		w1/sum*((refShape[i] - ctrlPoint[0])*rotMat+ctrlPoint[0]);
			//	ctrlPoint[0];
			//}
		}
	}


	void HangerAnimation::Anim_Vert_FromData(double t, std::vector<Vector3d>& pos, std::vector<Vector3d>& posLast, std::vector<Vector3d>& vel, std::vector<Vector3d>& force)
	{
		static int idx = 0;

		for(int i=0; i<pos.size(); i++)
		{
			double w1 = vertWeight[i*4+0];
			double w2 = vertWeight[i*4+1];
			double w3 = vertWeight[i*4+2];
			double w4 = vertWeight[i*4+3];

			if(w1>0.85 || w2>0.8|| w3>0.8)
			{
				double sum = w1 + w2 + w3;
				pos[i] = /*w1*((refShape[i] - ctrlPoint[0])+ctrlPoint[0]) + 
					w2*((refShape[i] - ctrlPoint[1])+ctrlPoint[1])+ 
					w3*((refShape[i] - ctrlPoint[2])+ctrlPoint[2])+*/
					w1/sum*((refShape[i] - ctrlPoint[0])*middleOri[idx] + middle[idx]) + 
					w2/sum*((refShape[i] - ctrlPoint[1]) + left[idx])+ 
					w3/sum*((refShape[i] - ctrlPoint[2]) + right[idx]);

			}

			//if(w4>0.8)
			//{
			//	pos[i] = /*w1*((refShape[i] - ctrlPoint[0])+ctrlPoint[0]) + 
			//		w2*((refShape[i] - ctrlPoint[1])+ctrlPoint[1])+ 
			//		w3*((refShape[i] - ctrlPoint[2])+ctrlPoint[2])+*/
			//		((refShape[i] - ctrlPoint[3])*middleOri[idx] + middle[idx]);
			//}

			//if(w1>0.5 )
			//{
			//	double sum = w1;
			//	pos[i] = /*w1*((refShape[i] - ctrlPoint[0])+ctrlPoint[0]) + 
			//		w2*((refShape[i] - ctrlPoint[1])+ctrlPoint[1])+ 
			//		w3*((refShape[i] - ctrlPoint[2])+ctrlPoint[2])+*/
			//		w1/sum*((refShape[i] - ctrlPoint[0]) + middle[idx]);

			//}
		}

		idx++;
	}

	void HangerAnimation::Process_Data()
	{
		int numFrame = motionData.size()/3;

		for(int i=0; i<numFrame; i++)
		{
			middle.push_back(motionData[i*3+0]);
			left.push_back(motionData[i*3+1]);
			right.push_back(motionData[i*3+2]);
		}

		for(int i=0; i<numFrame; i++)
		{
			Matrix3d left_tmp; 
			Matrix3d right_tmp;
			Matrix3d middle_tmp;

			middle_tmp.SetRow(1, orientData[i*9+0]);
			middle_tmp.SetRow(2, orientData[i*9+1]);
			middle_tmp.SetRow(3, orientData[i*9+2]);
			middleOri.push_back(middle_tmp.Inverse());

			left_tmp.SetRow(1, orientData[i*9+3]);
			left_tmp.SetRow(2, orientData[i*9+4]);
			left_tmp.SetRow(3, orientData[i*9+5]);
			leftOri.push_back(left_tmp.Inverse());

			right_tmp.SetRow(1, orientData[i*9+6]);
			right_tmp.SetRow(2, orientData[i*9+7]);
			right_tmp.SetRow(3, orientData[i*9+8]);
			rightOri.push_back(right_tmp.Inverse());

		}
	}
}		