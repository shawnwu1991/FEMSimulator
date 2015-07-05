#pragma once

#include "VecMatInterface.h"
#include "StdStrings.h"

#include "TetMesh.h"
#include "CorotatedLinearModel.h"
#include "RestShapeProblem.h"
#include "FEMIntegrator.h"
#include "nlopt.hpp"
#include <math.h>
#include "CentralGLWidget.h"

#include <vector>

namespace Libin
{

	class TrajParameterProblem : public RestShapeProblem
	{
	private:
		TrajParameterProblem(const TrajParameterProblem &);
		TrajParameterProblem &operator =(const TrajParameterProblem &);

	public:
		TrajParameterProblem(std::shared_ptr<TetMeshTopology> topology);
		virtual ~TrajParameterProblem(void);

		virtual bool Initialize(const Vector3d &gravity, const std::vector<double> &vertMass, const std::shared_ptr<TetMesh>& tetMesh);

		void ExportFrames(int index, const std::vector<Vector3d> &nodalPosition1, const std::vector<Vector3d> &nodalPosition2);

		bool LoadExample(const tstring &prefix, const tstring &suffix, int digitWidth, 
			int startidx, int step, int endidx, int numLineToSkip, double timestep);

		void ComputeExampleVelAccel(const Vector3d &gravity, const std::vector<double> &vertMass,
			const std::vector<std::shared_ptr<TetMesh>>& exampleShape,
			std::vector<std::vector<Vector3d>>& exampleSpeeds,
			std::vector<std::vector<Vector3d>>& exampleAccel,
			std::vector<std::vector<Vector3d>>& exampleForce);

		void InitializeNLopt();
		void UpdateNLopt();
		double ComputeShapeEnergy(const std::vector<Vector3d> &nodalPosition1, const std::vector<Vector3d> &nodalPosition2, double ratio, int trajIndex);
		double ComputeShapeEnergyReduced(double* exampleCoord, const std::vector<Vector3d> &nodalPosition1, const std::vector<Vector3d> &ref, double ratio, int trajIndex);

		std::shared_ptr<TetMesh> GetDynExampleShape(int frameId) const { return m_exampleShapesDyn[frameId]; }

		void EncodeVariable(const std::vector<double>& input, std::vector<double>& output);
		void DecodeVariable(const std::vector<double>& input, std::vector<double>& output);

		bool RetriveReferenceShape(std::vector<double>& curValue, std::vector<Vector3d>& _input, std::vector<Vector3d>& _output);
		double MatchTrajectory(std::vector<double>& curVale);
		void DrawObjFunction( std::vector<double>& _curValue );

		void InitModes();
		void ComputeDisplacement(std::vector<Vector3d> _ref);
		void ComputeModes();
		void ComputeTrajactoryMode(int startIdx, int endIdx);
		void ComputeReducedCoord(std::vector<Vector3d>& _disVec, std::vector<double>& _coord);

		void OutputEigenMode(std::vector<double>& _eigVec);
		void OutputReferenceShape(const std::vector<Vector3d>& input);

		void InitOptiOutput(const QString &filename);
		void InitObjFunOutput(const QString &filename);
		void InitWSOutput(const QString &filename);

		double WarmStart( std::vector<double>& _curValue );
		bool WarmStartCount();
		void WarmStartSample();
		void WarmStartSampleFast();
		double WarmStartLineSearch(int j, std::vector<int>& curSample, std::vector<double>& diff);
		double FindCandidateFrames(int startIdx, int endIdx, double time);

		double COV(int startIdx, std::vector<double>& data, double mean);
		int FindMinSample(int startIdx, std::vector<double>& data);

		double FindMin(int startIdx, int endIdx, std::vector<double>& data);
		double FindMax(int startIdx, int endIdx, std::vector<double>& data);

	public:
		QVector<GLint>     m_surfaceFace;
		std::vector<std::shared_ptr<TetMesh>> m_exampleShapesDyn;
		std::vector<int> m_frameSegDyn;
		std::vector<int> m_iterationNum;

		std::vector<std::vector<Vector3d>> m_exampleSpeeds;
		std::vector<std::vector<Vector3d>> m_exampleAccel;
		std::vector<std::vector<Vector3d>> m_exampleForce;
		std::vector<std::vector<Vector3d>> m_exampleDisplacement;

		std::vector<double> m_exampleTimeInterval;
		std::vector<double> m_iterationSetp;
		std::vector<double> m_frameError;

		std::vector<double> m_upBound;
		std::vector<double> m_lowBound;
		std::vector<double> m_gradientStep;
		std::vector<double> m_upBoundWS;
		std::vector<double> m_lowBoundWS;

		std::vector<double> m_E;
		std::vector<double> m_nu;

		std::vector<int> m_WSSample;
		
		std::vector<double> m_exampleShapeDynReduced;
		std::vector<std::vector<Vector3d>> m_EigenModes;
		std::vector<double> m_NaturalFrequencies;
		int					m_numModes;

		int					m_numSampleWS;
		double				m_stiffnessDampWS;
		double				m_massDampWS;

		bool				m_bOutputOpti;
		bool				m_bOutputObjFun;
		bool				m_bWarmStart;


		/*  variables  that  are  passed  to  KNITRO  */
		nlopt::opt m_NLopt;

		int m_maxIterationNum;
		int m_curState;

		std::shared_ptr<FEMIntegrator> m_pIntegrator;
		
		std::vector<Vector3d> m_refShape;
		std::vector<Vector3d>	m_candidateStaticExampleForce;

		int nu_index;
		int E_index;
		int m_numOptVariable;

		tofstream fout_score;
		tofstream fout_opti;
		tofstream fout_objfun;
		tofstream fout_WS;
	};

}
