#include "stdafx.h"
#include "GreenRubberAnimation.h"

#pragma comment (lib, "QPSolver.lib")

#include <iomanip>
#include <numeric>
#include <algorithm>
#include <functional>
#include <fstream>
#include <string>

namespace Libin
{

	GreenRubberAnimation::GreenRubberAnimation()
	{
		curTime = 0.0;
	}

	GreenRubberAnimation::~GreenRubberAnimation(void)
	{
	}

	bool GreenRubberAnimation::LoadWeight(const QString &pathdir)
	{
		// load node coord
		tifstream ifStr;
		ifStr.open((pathdir+ "Model/weightVert.txt").utf16());	

		tstring line;
		while (std::getline(ifStr, line))
		{
			tistringstream sstr(line);
			double tmpValue;
			for(int j=0; j<6; j++)
			{
				sstr >> tmpValue;
				vertWeight.push_back(tmpValue);
			}
		}

		ifStr.close();
		return true;
	}


	bool GreenRubberAnimation::LoadCtrlPoint(const QString &pathdir)
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

	bool GreenRubberAnimation::LoadRefShape(const QString &pathdir)
	{
		// load ref shape
		tifstream ifStr;
		ifStr.open((pathdir+ "Model/rot_curve_cushion.vert").utf16());	
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


	void GreenRubberAnimation::Anim_Vert(double t, std::vector<Vector3d>& pos, std::vector<Vector3d>& posLast, std::vector<Vector3d>& vel, std::vector<Vector3d>& force)
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
	/*	if(cos(2*angle)>0.0)
		{
			displace += 0.07*cos(2*angle)*t;	
			angle1 = -0.25*cos(2*angle)-0.1;
			angle2 = 0.00000015*cos(2*angle);
		}
		if(cos(2*angle)<0.0)
		{
			displace2 += -0.07*cos(2*angle)*t;
			angle2 = 0.25*cos(2*angle)-0.1;
			angle1 = -0.000000015*cos(2*angle);
			first = false;
		}*/

		angle1 = 0.8*cos(2*angle)/*-0.1*/;
		angle2 = -0.8*cos(2*angle)/*-0.1*/;
		
		if(cos(2*angle)>0.0)
		{
		//angle1 = 0.8*cos(2*angle)/*-0.1*/;
		displace += 0.035*cos(2*angle)*t;
		}
		else
		{
		//angle2 = -0.8*cos(2*angle)/*-0.1*/;
		
		displace2 += -0.035*cos(2*angle)*t;
		}

		displacey += 0.03*cos(4*angle)*t;
		//displacey = 0.1*cos(4*angle)*t;
		displacey = 0.01*cos(4*angle);

		Matrix3d rotMat;
		Matrix3d rotMat2;
		rotMat.MakeEulerXYZ(angle1, 0.0, -0.6*angle1);
		rotMat2.MakeEulerXYZ(angle2, 0.0, -0.6*angle1);

		Matrix3d rotMat3;
		Matrix3d rotMat4;
		rotMat3.MakeEulerXYZ(angle1, 0.0, 0.2*angle1);
		rotMat4.MakeEulerXYZ(angle1, 0.0, 0.2*angle1);
		
		for(int i=0; i<pos.size(); i++)
		{
			double w1 = vertWeight[i*6+0];
			double w2 = vertWeight[i*6+1];
			double w3 = vertWeight[i*6+2];
			double w4 = vertWeight[i*6+3];
			double w5 = vertWeight[i*6+4];
			double w6 = vertWeight[i*6+5];

			if(w1>0.8|| w4>0.8)
			{
				double sum = w1 + w4;
				pos[i] = /*w1*((refShape[i] - ctrlPoint[0])+ctrlPoint[0]) + 
					w2*((refShape[i] - ctrlPoint[1])+ctrlPoint[1])+ 
					w3*((refShape[i] - ctrlPoint[2])+ctrlPoint[2])+*/
					w1/sum*((refShape[i] - ctrlPoint[0])*rotMat+ctrlPoint[0]  + Vector3d(0,  displacey,  displace)) + 
					w4/sum*((refShape[i] - ctrlPoint[3])*rotMat2+ctrlPoint[3] + Vector3d(0,  displacey,  displace2));

				//pos[i] = pos[i] - Vector3d(0.0, -1.0, 0.0);
			}

			if(w5>0.98 || w6>0.98)
			{
				double sum = w5 + w6;
				pos[i] = /*w1*((refShape[i] - ctrlPoint[0])+ctrlPoint[0]) + 
					w2*((refShape[i] - ctrlPoint[1])+ctrlPoint[1])+ 
					w3*((refShape[i] - ctrlPoint[2])+ctrlPoint[2])+*/
					w5/sum*((refShape[i] - ctrlPoint[4])+ctrlPoint[4] + Vector3d(0, displacey, 0.5*(displace+displace2))) + 
					w6/sum*((refShape[i] - ctrlPoint[5])+ctrlPoint[5] + Vector3d(0, displacey, 0.5*(displace+displace2)));
			}

			static double accu_time = 0.0;
		    accu_time += t;
			double angle3 = 1.5 * sin(accu_time*10);

			//std::cout << "angle3: " << angle3 * 180.0/3.14 << std::endl;
		
			Matrix3d rotMat3;
		    rotMat3.MakeEulerXYZ(0.9, 0.0, 0.0);
	
			//if(w1>0.8)
			//{
			//	double sum = w1 ;
			//	pos[i] = /*w1*((refShape[i] - ctrlPoint[0])+ctrlPoint[0]) + 
			//	//	w2*((refShape[i] - ctrlPoint[1])+ctrlPoint[1])+ 
			//	//	w3*((refShape[i] - ctrlPoint[2])+ctrlPoint[2])+*/
			//		w1/sum*((refShape[i] - ctrlPoint[0])*rotMat+ctrlPoint[0]);
			//}
		}
	}
}		