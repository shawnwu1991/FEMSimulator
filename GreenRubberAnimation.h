#pragma once


#include "VecMatInterface.h"
#include <vector>
#include <fstream>
#include "StdStrings.h"
#include <QString>

namespace Libin
{

	class GreenRubberAnimation
	{
	public:
		GreenRubberAnimation();
		virtual ~GreenRubberAnimation(void);

		bool LoadWeight(const QString &pathdir);
		bool LoadCtrlPoint(const QString &pathdir);
		bool LoadRefShape(const	QString &pathdir);

		void Anim_Vert(double t, std::vector<Vector3d>& pos, std::vector<Vector3d>& posLast, std::vector<Vector3d>& vel, std::vector<Vector3d>& force);

	public:
		Vector3d bodyFrame;

		std::vector<Vector3d> refShape;
		std::vector<double> vertWeight;
		std::vector<Vector3d> ctrlPoint;

		std::vector<double> motionData;

		double curTime;

	};

}
