#pragma once


#include "VecMatInterface.h"
#include <vector>
#include <fstream>
#include "StdStrings.h"
#include <QString>

namespace Libin
{

	class HangerAnimation
	{
	public:
		HangerAnimation();
		virtual ~HangerAnimation(void);

		bool LoadWeight(const QString &pathdir);
		bool LoadCtrlPoint(const QString &pathdir);
		bool LoadRefShape(const	QString &pathdir);
		bool LoadAnimData(const QString &pathdir);

		void Anim_Vert(double t, std::vector<Vector3d>& pos, std::vector<Vector3d>& posLast, std::vector<Vector3d>& vel, std::vector<Vector3d>& force);
		void Anim_Vert_FromData(double t, std::vector<Vector3d>& pos, std::vector<Vector3d>& posLast, std::vector<Vector3d>& vel, std::vector<Vector3d>& force);

		void Process_Data();

	public:
		Vector3d bodyFrame;

		std::vector<Vector3d> refShape;
		std::vector<double> vertWeight;
		std::vector<Vector3d> ctrlPoint;

		std::vector<Vector3d> motionData;
		std::vector<Vector3d> orientData;

		std::vector<Vector3d> left;
		std::vector<Vector3d> right;
		std::vector<Vector3d> middle;

		std::vector<Matrix3d> leftOri;
		std::vector<Matrix3d> rightOri;
		std::vector<Matrix3d> middleOri;


	/*	std::vector<Vector3d> leftKnee;
		std::vector<Vector3d> rightKnee;
		std::vector<Vector3d> pelvi;
		std::vector<double> leftKneeAngle;
		std::vector<double> rightKneeAngle;*/

	/*	Vector3d LF_dis;
		Vector3d RF_dis;
		Vector3d LB_dis;
		Vector3d RB_dis;*/


		double curTime;

	};

}
