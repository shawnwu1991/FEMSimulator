#include "stdafx.h"
#include "RestShapeProblem.h"
#include "TetMesh.h"
#include "CorotatedLinearModelMultiThreaded.h"

#include "..\QPSolver\QPSolver.h"


#pragma comment (lib, "QPSolver.lib")

#include <iomanip>
#include <numeric>
#include <algorithm>
#include <functional>

namespace Libin
{

	RestShapeProblem::RestShapeProblem(std::shared_ptr<TetMeshTopology> topology)
		: m_topology(topology)
	{
		m_numElement = m_topology->GetNumElement();
		m_numNode    = m_topology->GetNumVertices();	

		m_materials = std::make_shared<TetMeshMaterials>();
		m_totalElmMaterialStiffness.resize(m_numElement);
	}


	RestShapeProblem::~RestShapeProblem(void)
	{
	}

	bool RestShapeProblem::LoadStaticFrame(const tstring &frame, int numLineToSkip, std::shared_ptr<TetMesh>& staticFrame)
	{
		tstringstream fn;
		fn << frame;
		std::tcout << TEXT("Loading ") << fn.str() << TEXT("... ");

		std::shared_ptr<TetMesh> tetMesh = std::make_shared<TetMesh>(m_topology, std::shared_ptr<TetMeshVertices>(), m_materials);
		if (!tetMesh->LoadVertices(fn.str(), numLineToSkip))
			return false;
		if (m_numNode != (int)tetMesh->GetVertices().size())
		{
			std::tcout << TEXT("Wrong vertex count: ") << tetMesh->GetVertices().size() << TEXT(" ") << m_numNode << TEXT(" needed!") << std::endl;
			return false;
		}  

		staticFrame = tetMesh;
		std::tcout << TEXT("Done!") << std::endl;

		return true;
	}


	void RestShapeProblem::ComputeExampleForce(const Vector3d &gravity, const std::vector<double> &vertMass)
	{
		m_staticExampleForce.clear();
		m_staticExampleForce.resize(m_numNode, Vector3d::ZERO);
		m_staticNodalForce.clear();
		m_staticNodalForce.resize(m_numNode, Vector3d::ZERO);
		m_staticForceRes.clear();
		m_staticForceRes.resize(m_numNode, Vector3d::ZERO);

		
		for (int vi = 0; vi < m_numNode; ++vi)
			m_staticExampleForce[vi] = -gravity * vertMass[vi];
	}


	bool RestShapeProblem::Initialize(const Vector3d &gravity, const std::vector<double> &vertMass,  const std::shared_ptr<TetMesh>& tetMesh)
	{
		// define the corotational FEM model for single or multi thread mode
		m_model = std::make_shared<CorotatedLinearModel>(m_staticShape->GetTopology());
		std::tcout  << TEXT(" example shapes loaded!") << std::endl;

		// compute total example force and acceleration
		ComputeExampleForce( gravity, vertMass);

		// computer the init force residule
		double initForceRes = 0;
		for (int vi = 0; vi < m_numNode; ++vi)
			initForceRes += m_staticExampleForce[vi].SquaredLength();
		initForceRes = sqrt(initForceRes);
		std::tcout << TEXT("Zero force res: ") << std::setprecision(18) << initForceRes << std::endl;

		return true;
	}


	void RestShapeProblem::Update(const std::vector<Vector3d> &restShape, int type, int start /*= 1*/, int end /* = -1*/)
	{
		double *totalElmMaterialStiffness = reinterpret_cast<double *>(m_totalElmMaterialStiffness.data());
		double *staticNodalForce = reinterpret_cast<double *>(m_staticNodalForce.data());

		int stiffnessDim = 144 * m_totalElmMaterialStiffness.size();
		int forceDim = 3 * m_staticNodalForce.size();

		memset(totalElmMaterialStiffness, 0, sizeof(std::array<double, 12*12>) * m_totalElmMaterialStiffness.size());
		memset(staticNodalForce,  0, sizeof(Vector3d) * m_staticNodalForce.size());
		
		std::shared_ptr<TetMeshVertices> restShapeVertices = std::make_shared<TetMeshVertices>();
		restShapeVertices->GetVertices() = restShape;
		TetMesh restTetMesh(m_topology, restShapeVertices, m_materials);
		restTetMesh.ComputeVolumeMass();

		class Task : public QRunnable
		{
			std::shared_ptr<CorotatedLinearModel> m_model;
			TetMesh &m_restTetMesh;
			const std::vector<Vector3d> &m_exampleShape;
			const std::vector<Vector3d> &m_exampleForce;
			std::vector<Vector3d> &m_forceRes;
			std::vector<Vector3d> &m_nodalForce;

			int m_type;
		public:
			Task(std::shared_ptr<CorotatedLinearModel> model, TetMesh &restTetMesh, 
				const std::vector<Vector3d> &exampleShape, std::vector<Vector3d> &nodalForce,
				const std::vector<Vector3d> &exampleForce, std::vector<Vector3d> &forceRes,
				int type
				)
				: m_model(model), m_restTetMesh(restTetMesh), m_exampleShape(exampleShape), m_nodalForce(nodalForce)
				, m_exampleForce(exampleForce), m_forceRes(forceRes)
				, m_type(type)
			{
			}

			void run(void)
			{
				m_model->InitializeUndeformedStiffnessMatrix(&m_restTetMesh);
				if (m_type & RestShapeProblem::UT_StiffnessMatrix)
					m_model->UpdateForceMaterialStiffnessMatrix(m_exampleShape);
				if (m_type & RestShapeProblem::UT_NodalForce)
				{
					m_model->ComputeElasticForce(m_exampleShape, m_nodalForce, CorotatedLinearModel::StiffneeMatrix);
					for (size_t i = 0; i < m_exampleShape.size(); ++i)
						m_forceRes[i] = m_exampleForce[i] + m_nodalForce[i];
				}
			}
		};

		Task(m_model, restTetMesh, m_staticShape->GetVertices(), 
			m_staticNodalForce, m_staticExampleForce, m_staticForceRes, type).run();

		if (type & UT_StiffnessMatrix)
		{
			// sums up stiffness matrix
			const double *elmMaterialStiffness = reinterpret_cast<const double *>(m_model->GetElementMaterialStiffnessMatrix().data());
			for (int di = 0; di < stiffnessDim; ++di)
				totalElmMaterialStiffness[di] += elmMaterialStiffness[di];

			for (int di = 0; di < stiffnessDim; ++di)
				totalElmMaterialStiffness[di] *= -1.0;
		}    

		if (type & UT_NodalForce)
		{
			for (int di = 0; di < m_staticNodalForce.size(); ++di)
				m_staticNodalForce[di] *= -1;
		}
	}

	void RestShapeProblem::SetMaterial(const TetMeshMaterials* mtPtr)
	{
		m_materials->m_ctrlE = mtPtr->m_ctrlE;
		m_materials->m_ctrlNu = mtPtr->m_ctrlNu;
		m_materials->m_ctrlWeight = mtPtr->m_ctrlWeight;
		m_materials->m_numCtrlPoint = mtPtr->m_numCtrlPoint;

		m_materials->m_elmMaterial.resize(mtPtr->m_elmMaterial.size());
		for(int i=0; i<mtPtr->m_elmMaterial.size();i++)
			m_materials->m_elmMaterial[i] = mtPtr->m_elmMaterial[i];

	}
}