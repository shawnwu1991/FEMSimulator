#include "stdafx.h"
#include "Animation.h"

#pragma comment (lib, "QPSolver.lib")

#include <iomanip>
#include <numeric>
#include <algorithm>
#include <functional>
#include <fstream>
#include <string>

namespace Libin
{

	Animation::Animation()
	{
		curTime = 0.0;
	}

	Animation::~Animation(void)
	{
	}

	bool Animation::LoadBody(const QString &pathdir)
	{
		// load node coord
		tifstream ifStr;
		ifStr.open((pathdir+ "Model/bodyCoord.txt").utf16());	

		Vector3d tmpCoord;
		while (ifStr >> tmpCoord.X() >> tmpCoord.Y() >> tmpCoord.Z())
		{
			bodyCoord.push_back(tmpCoord);
			std::cout << "body coord: " << bodyCoord.size() << " " << bodyCoord[bodyCoord.size()-1].X() << " " <<  bodyCoord[bodyCoord.size()-1].Y() 
				<< " " <<  bodyCoord[bodyCoord.size()-1].Z() << " " <<  std::endl;
		}
		ifStr.close();

		// load node coord
		ifStr.open((pathdir+ "Model/bodyNode.txt").utf16());	

		int tmpNode;
		while (ifStr >> tmpNode)
		{
			bodyNode.push_back(tmpNode);
			std::cout << "body node: " << bodyNode.size() << " " << bodyNode[bodyNode.size()-1] << " ";
		}
		std::cout << std::endl;
		ifStr.close();

		// load frame coord
		ifStr.open((pathdir+ "Model/bodyFrame.txt").utf16());	
		ifStr >> bodyFrame.X() >> bodyFrame.Y() >> bodyFrame.Z();
		std::cout << "body frame: " << bodyFrame.X() << " " <<  bodyFrame.Y()  << " " <<  bodyFrame.Z() << " " <<  std::endl;
		ifStr.close();

		return true;
	}

	bool Animation::LoadFoot(const QString &pathdir)
	{
		// load node coord
		tifstream ifStr;
		ifStr.open((pathdir+ "Model/footCoord.txt").utf16());	

		Vector3d tmpCoord;
		while (ifStr >> tmpCoord.X() >> tmpCoord.Y() >> tmpCoord.Z())
		{
			footCoord.push_back(tmpCoord);
			std::cout << "body coord: " << footCoord.size() << " " << footCoord[footCoord.size()-1].X() << " " <<  footCoord[footCoord.size()-1].Y() 
				<< " " <<  footCoord[footCoord.size()-1].Z() << " " <<  std::endl;
		}
		ifStr.close();

		// load node coord
		ifStr.open((pathdir+ "Model/footNode.txt").utf16());	

		int tmpNode;
		while (ifStr >> tmpNode)
		{
			footNode.push_back(tmpNode);
			std::cout << "body node: " << footNode.size() << " " << footNode[footNode.size()-1] << " ";
		}
		std::cout << std::endl;
		ifStr.close();

		// load node mode
		ifStr.open((pathdir+ "Model/mode_3.txt").utf16());	

		while (ifStr >> tmpCoord.X() >> tmpCoord.Y() >> tmpCoord.Z())
		{
			footMode.push_back(tmpCoord);
			//std::cout << "body coord: " << footMode.size() << " " << footMode[footMode.size()-1].X() << " " <<  footMode[footMode.size()-1].Y() 
			//	<< " " <<  footMode[footMode.size()-1].Z() << " " <<  std::endl;
		}
		ifStr.close();

		// load ref shape
		ifStr.open((pathdir+ "Model/rot_curve_cushion.vert").utf16());	
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

	void Animation::Anim_BodyFrame( double t)
	{
		curTime += t;

		cur_bodyFrame = bodyFrame;
		//cur_bodyFrame.Y() += sin(curTime*200)*t;
		//cur_bodyFrame.X() += 10*sin(curTime*200)*t;
		//cur_bodyFrame.Z() += 10*sin(curTime*50)*t;
		//cur_bodyFrame.Z() += 0.1*t;

		curMode = footMode;
		for(int i=0; i<footMode.size(); i++)
			curMode[i] = t*sin(curTime*50)*footMode[i];
	}

	void Animation::Anim_BodyNode(double t, std::vector<Vector3d>& pos, std::vector<Vector3d>& posLast, std::vector<Vector3d>& vel)
	{
		//for (int i=0; i<bodyNode.size(); i++)
		//{
		//	//vel[bodyNode[i]] = cur_bodyFrame;
		//	pos[bodyNode[i]] = cur_bodyFrame + bodyCoord[i];
		//	vel[bodyNode[i]] = (pos[bodyNode[i]] - posLast[bodyNode[i]])/t;
		//}

		for (int i=0; i<footNode.size(); i++)
		{
			pos[footNode[i]] = refShape[footNode[i]] + 2.0*curMode[footNode[i]];
			vel[footNode[i]] = (pos[footNode[i]] - posLast[footNode[i]])/t;
		}
	}

}		