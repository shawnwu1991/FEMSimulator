#pragma once


#include "VecMatInterface.h"
#include <vector>
#include <fstream>
#include "StdStrings.h"
#include <QString>

namespace Libin
{

	class Animation
	{
	public:
		Animation();
		virtual ~Animation(void);

		bool LoadBody(const QString &pathdir);
		bool LoadFoot(const QString &pathdir);
		void Anim_BodyFrame( double t);
		void Anim_BodyNode(double t, std::vector<Vector3d>& pos, std::vector<Vector3d>& posLast, std::vector<Vector3d>& vel);

	public:
		Vector3d bodyFrame;

		std::vector<Vector3d> bodyCoord;
		std::vector<Vector3d> footCoord;

		std::vector<int> bodyNode;
		std::vector<int> footNode;

		std::vector<Vector3d> footMode;
		std::vector<Vector3d> curMode;
		std::vector<Vector3d> refShape;

		Vector3d cur_bodyFrame;

		double curTime;

	};

}
