#include "stdafx.h"
#include "FEMIntegrator.h"

#include "TetMesh.h"
#include "CorotatedLinearModel.h"
#include "VegaFEMInterface.h"
#include "RestShapeProblem.h"

#include <functional>

namespace Libin
{

	FEMIntegrator::FEMIntegrator(CorotatedLinearModel *pModel, TetMesh *tetMesh,
		std::shared_ptr<RestShapeProblem> restShapeProblem /*= std::shared_ptr<RestShapeProblem>()*/)
		: m_pModel(pModel)
		, m_pTetMesh(tetMesh)
		, m_dampingStiffnessCoef(0.0)
		, m_dampingMassCoef(0.0)
		, m_gravity(Vector3d::ZERO)
		, m_integratorId(0)
		, m_flagNeedReInitInternalSolver(true)
		, m_restShapeProblem(restShapeProblem)
	{
		m_nodalPosition = tetMesh->GetVertices();
		m_nodalVelocity.resize(m_nodalPosition.size(), Vector3d::ZERO);
		m_nodalForce.resize(m_nodalPosition.size(), Vector3d::ZERO);

		m_nodalPositionLast = tetMesh->GetVertices();
		m_nodalVelocityLast.resize(m_nodalPosition.size(), Vector3d::ZERO);

		m_vertMass = tetMesh->GetVertexMass();

		m_externalForce.resize(m_nodalPosition.size(), Vector3d::ZERO);
		m_gravityForce.resize(m_nodalPosition.size(), Vector3d::ZERO);
		m_airForce.resize(m_nodalPosition.size(), Vector3d::ZERO);
		m_zeroForce.resize(m_nodalPosition.size(), Vector3d::ZERO);

		//VegaFEMInterface::Instance(m_integratorId)->Initialize(
		//    pModel->GetTopology(),
		//    m_vertMass,
		//    std::vector<int>());
	}


	FEMIntegrator::~FEMIntegrator(void)
	{
	}

	void FEMIntegrator::ClearInternalForce(void)
	{
		std::fill(m_nodalForce.begin(), m_nodalForce.end(), Vector3d::ZERO);
	}

	void FEMIntegrator::ClearExternalForce(void)
	{
		std::fill(m_externalForce.begin(), m_externalForce.end(), Vector3d::ZERO);
	}

	void FEMIntegrator::AddExternalForce(const std::vector<Vector3d> &forces)
	{
		std::transform(forces.begin(), forces.end(), m_externalForce.begin(), m_externalForce.begin(), std::plus<Vector3d>());
	}


	void FEMIntegrator::SetGravity(const Vector3d &g)
	{
		m_gravity = g;
		for (size_t i = 0; i < m_vertMass.size(); ++i)
			m_gravityForce[i] = m_vertMass[i] * g;
	}

	void FEMIntegrator::SetDampingParameters(double dampingStiffnessCoef, double dampingMassCoef)
	{
		m_dampingStiffnessCoef = dampingStiffnessCoef;
		m_dampingMassCoef = dampingMassCoef;
	}

	void FEMIntegrator::ConstrainNodesInRange(double range[6], TetMesh *tetMesh)
	{
		std::tcout << TEXT("Checking constrained nodes ...") << std::endl;
		auto &vertices = tetMesh->GetVertices();
		m_constrainedDof.clear();
		for (size_t i = 0; i < vertices.size(); ++i)
		{
			auto &vt = vertices[i];
			if (vt[0] >= range[0] && vt[0] <= range[1] &&
				vt[1] >= range[2] && vt[1] <= range[3] &&
				vt[2] >= range[4] && vt[2] <= range[5])
			{
				m_constrainedDof.push_back(i * 3);
				m_constrainedDof.push_back(i * 3 + 1);
				m_constrainedDof.push_back(i * 3 + 2);
				std::tcout << i << TEXT(" ");
			}
		}
		std::tcout << std::endl;

		std::sort(m_constrainedDof.begin(), m_constrainedDof.end());
		m_constrainedDofPosBuf.resize(m_constrainedDof.size());
		m_constrainedDofVelBuf.resize(m_constrainedDof.size());
		m_flagNeedReInitInternalSolver = true;
	}

	void FEMIntegrator::ConstrainNodesByIndex(std::vector<int> nodeIdx)
	{
		std::tcout << TEXT("Constrained node: ") << std::endl;
		for (auto ni : nodeIdx)
		{
			m_constrainedDof.push_back(ni * 3);
			m_constrainedDof.push_back(ni * 3 + 1);
			m_constrainedDof.push_back(ni * 3 + 2);
			std::tcout << ni << TEXT(" ");
		}
		std::tcout << std::endl;

		std::sort(m_constrainedDof.begin(), m_constrainedDof.end());
		m_constrainedDofPosBuf.resize(m_constrainedDof.size());
		m_constrainedDofVelBuf.resize(m_constrainedDof.size());
		m_flagNeedReInitInternalSolver = true;
		std::tcout << TEXT("Finish Constrained node: ") << std::endl;
	}

	void FEMIntegrator::ControledNodesByIndex(std::vector<int> nodeIdx)
	{
		std::tcout << TEXT("Constrained node: ") << std::endl;
		for (auto ni : nodeIdx)
		{
			m_controledNode.push_back(ni);
			std::tcout << ni << TEXT(" ");
		}
		std::tcout << std::endl;

		//std::sort(m_constrainedDof.begin(), m_constrainedDof.end());
		//m_constrainedDofPosBuf.resize(m_constrainedDof.size());
		//m_constrainedDofVelBuf.resize(m_constrainedDof.size());
		//m_flagNeedReInitInternalSolver = true;
	}

	void FEMIntegrator::SetState(const std::vector<Vector3d> &nodalPosition, const std::vector<Vector3d> &nodalVelocity /*= std::vector<Vector3d>()*/)
	{
		if (m_nodalPosition.size() > nodalPosition.size())
		{
			std::tcout << TEXT("Warning! Insufficient positions are given!") << std::endl;
			std::copy(nodalPosition.begin(), nodalPosition.end(), m_nodalPosition.begin());
		}
		else
			std::copy(nodalPosition.begin(), nodalPosition.begin() + m_nodalPosition.size(), m_nodalPosition.begin());

		if (nodalVelocity.empty())
			std::fill(m_nodalVelocity.begin(), m_nodalVelocity.end(), Vector3d::ZERO);
		else if (m_nodalVelocity.size() > nodalVelocity.size())
		{
			std::tcout << TEXT("Warning! Insufficient velocities are given!") << std::endl;
			std::copy(nodalVelocity.begin(), nodalVelocity.end(), m_nodalVelocity.begin());
		}
		else
			std::copy(nodalVelocity.begin(), nodalVelocity.begin() + m_nodalVelocity.size(), m_nodalVelocity.begin());

		std::fill(m_nodalForce.begin(), m_nodalForce.end(), Vector3d::ZERO);

		m_nodalPositionLast = m_nodalPosition;
		m_nodalVelocityLast = m_nodalVelocity;
	}

	void FEMIntegrator::SymplecticEulerStep(double t)
	{
		m_nodalPositionLast = m_nodalPosition;
		m_nodalVelocityLast = m_nodalVelocity;

		// update elastic forces
		m_pModel->ResetAllFlags();
		m_pModel->ComputeElasticForce(m_nodalPosition, m_nodalForce, CorotatedLinearModel::StressTensor);

		// Ma + f_e = fext;
		size_t cdof = m_constrainedDof.size();

		double *pos = reinterpret_cast<double *>(m_nodalPosition.data());
		double *vel = reinterpret_cast<double *>(m_nodalVelocity.data());
		for (size_t ci = 0; ci < cdof; ++ci)
		{
			m_constrainedDofPosBuf[ci] = pos[m_constrainedDof[ci]];
			m_constrainedDofVelBuf[ci] = vel[m_constrainedDof[ci]];
		}

		auto &mass = m_vertMass;
		size_t nvert = m_nodalPosition.size();
		for (size_t vi = 0; vi < nvert; ++vi)
		{
			Vector3d a = m_nodalForce[vi];
			a = -a;
			a += m_externalForce[vi];
			a /= mass[vi];
			a += m_gravity;

			m_nodalVelocity[vi] += a * t;
			m_nodalPosition[vi] += m_nodalVelocity[vi] * t;
		}

		for (size_t ci = 0; ci < cdof; ++ci)
		{
			pos[m_constrainedDof[ci]] = m_constrainedDofPosBuf[ci];
			vel[m_constrainedDof[ci]] = m_constrainedDofVelBuf[ci];
		}
	}

	bool FEMIntegrator::BackwardEulerStep(double t)
	{
		//RotateZ();
		//Drag();
		//Stretch();
		//m_pDinosaruAnimation->Anim_Vert(t, m_nodalPosition, m_nodalPositionLast, m_nodalVelocity, m_externalForce);
		//m_pGreenAnimation->Anim_Vert(t, m_nodalPosition, m_nodalPositionLast, m_nodalVelocity, m_externalForce);
		//m_pHangerAnimation->Anim_Vert_FromData(t, m_nodalPosition, m_nodalPositionLast, m_nodalVelocity, m_externalForce);
		//m_pHangerAnimation->Anim_Vert(t, m_nodalPosition, m_nodalPositionLast, m_nodalVelocity, m_externalForce);

		m_nodalPositionLast = m_nodalPosition;
		m_nodalVelocityLast = m_nodalVelocity;

		//m_pAnimation->Anim_BodyFrame(t);
		//m_pAnimation->Anim_BodyNode(t, m_nodalPosition, m_nodalPositionLast, m_nodalVelocity);

		size_t cdof = m_constrainedDof.size();

		double *pos = reinterpret_cast<double *>(m_nodalPosition.data());
		double *vel = reinterpret_cast<double *>(m_nodalVelocity.data());
		for (size_t ci = 0; ci < cdof; ++ci)
		{
			m_constrainedDofPosBuf[ci] = pos[m_constrainedDof[ci]];
			m_constrainedDofVelBuf[ci] = vel[m_constrainedDof[ci]];
		}


		if (m_flagNeedReInitInternalSolver)
		{
			VegaFEMInterface::Instance(m_integratorId)->Initialize(
				m_pModel->GetTopology(),
				m_vertMass,
				m_constrainedDof);
			m_flagNeedReInitInternalSolver = false;
		}

		m_pModel->ResetAllFlags();
		m_pModel->UpdateForceStiffnessMatrix(m_nodalPosition);
		m_pModel->ComputeElasticForce(m_nodalPosition, m_nodalForce, CorotatedLinearModel::StiffneeMatrix);
		auto &elmStiffness = m_pModel->GetElementStiffnessMatrix();

		for (size_t i = 0; i < m_addedForceVert.size(); ++i)
			m_externalForce[m_addedForceVert[i]] = m_addedForces[i];

		VegaFEMInterface::StepTask task = {            
			m_nodalPosition,        // std::vector<Vector3d> &nodalPosition;
			m_nodalVelocity,        // std::vector<Vector3d> &nodalVelocity;
			m_nodalForce,           // const std::vector<Vector3d> &nodalForce;
			m_externalForce,        // const std::vector<Vector3d> &externalForce;
			m_gravityForce,         // const Vector3d &g;
			elmStiffness,           // const std::vector<std::array<double, 12*12>> &elmStiffness;
			m_dampingStiffnessCoef, // double dampingStiffnessCoef;
			m_dampingMassCoef       // double dampingMassCoef;
		};
		bool flag = VegaFEMInterface::Instance(m_integratorId)->DoTimeStep(t, task);

		for (size_t ci = 0; ci < cdof; ++ci)
		{
			pos[m_constrainedDof[ci]] = m_constrainedDofPosBuf[ci];
			vel[m_constrainedDof[ci]] = m_constrainedDofVelBuf[ci];
		}
		//ConstraintEndPoint(t);

		//m_pDinosaruAnimation->Anim_Vert(t, m_nodalPosition, m_nodalPositionLast, m_nodalVelocity);

		return flag;
	}

	void FEMIntegrator::ConstraintEndPoint(double t)
	{
		static std::vector<Vector3d> desired_pos;
		desired_pos.resize(4);
		desired_pos[0] = Vector3d(-0.276,  0.459,  -0.01) + Vector3d(0.2, -0.1, 0.02);
		desired_pos[1] = Vector3d(-0.255,  0.454,  -0.01) + Vector3d(0.2, -0.1, 0.02);
		desired_pos[2] = Vector3d(-0.276,  0.459,  0.01)  + Vector3d(0.2, -0.1, 0.02);
		desired_pos[3] = Vector3d(-0.255,  0.454,  0.01)  + Vector3d(0.2, -0.1, 0.02);

		static std::vector<Vector3d> translation;
		translation.resize(4);
		translation[0] = Vector3d(0.2, -0.1, 0.02);
		translation[1] = Vector3d(0.2, -0.1, 0.02);
		translation[2] = Vector3d(0.2, -0.1, 0.02);
		translation[3] = Vector3d(0.2, -0.1, 0.02);

		/*double *pos = reinterpret_cast<double *>(m_nodalPosition.data());
		double *vel = reinterpret_cast<double *>(m_nodalVelocity.data());*/
		static int iter = 1;

		if(iter<10000)
		{
			double ratio = (double)1/10000.0;
			m_nodalPosition[112] = m_nodalPosition[112] + translation[0]*ratio;
			m_nodalPosition[113] = m_nodalPosition[113] + translation[1]*ratio;
			m_nodalPosition[114] = m_nodalPosition[114] + translation[2]*ratio;
			m_nodalPosition[115] = m_nodalPosition[115] + translation[3]*ratio;

			m_nodalVelocity[112] = translation[0]*ratio/t;
			m_nodalVelocity[113] = translation[1]*ratio/t;
			m_nodalVelocity[114] = translation[2]*ratio/t;
			m_nodalVelocity[115] = translation[3]*ratio/t;
		}
		else
		{
			std::cout << "iterate 100 times" << std::endl;
			m_nodalPosition[112] = m_nodalPositionLast[112];
			m_nodalPosition[113] = m_nodalPositionLast[113];
			m_nodalPosition[114] = m_nodalPositionLast[114];
			m_nodalPosition[115] = m_nodalPositionLast[115];

			m_nodalVelocity[112] = Vector3d(0.0, 0.0, 0.0);
			m_nodalVelocity[113] = Vector3d(0.0, 0.0, 0.0);
			m_nodalVelocity[114] = Vector3d(0.0, 0.0, 0.0);
			m_nodalVelocity[115] = Vector3d(0.0, 0.0, 0.0);
		}
		iter ++;
	}


	void FEMIntegrator::RotateZ()
	{
		//for(int i=0; i<m_controledNode.size(); i++)
		//{
		//	int idx = m_controledNode[i];

		//	Vector3d radius = m_nodalPosition[idx];
		//	radius.Z() = 0.0;
		//	radius.Y() = radius.Y() - 0.15;

		//	Vector3d force = radius.UnitCross(Vector3d(0,0,1));
		//	m_externalForce[idx] = m_externalForce[idx] + force * 40.0;

		//	//std::cout << "external force: " << force.X() << " " << force.Y() <<  " " << force.Z() << std::endl;
		//}

		int idx = m_controledNode[0];
	
		Vector3d radius = m_nodalPosition[1175] - m_nodalPosition[idx];
		Vector3d force = radius*0.01;
		m_externalForce[idx] = m_externalForce[idx] + 10.0*force;

		m_nodalPosition[idx] = m_nodalPosition[1175];
		m_nodalPosition[1106] = m_nodalPosition[1139];
		m_nodalPosition[1178] = m_nodalPosition[1211];
	}

	void FEMIntegrator::Drag()
	{
		std::vector<Vector3d> desirePoint(2);
		desirePoint[0] = Vector3d(-0.299, 0.4, 0.2);
		desirePoint[1] = Vector3d(0.299, 0.4, 0.2);

		for(int i=0; i<m_controledNode.size(); i++)
		{
			int idx = m_controledNode[i];

			Vector3d deisrePoint = desirePoint[i];
			Vector3d force = deisrePoint/deisrePoint.Normalize();
			m_externalForce[idx] = m_externalForce[idx] + force * 40.0;

			//std::cout << "external force: " << force.X() << " " << force.Y() <<  " " << force.Z() << std::endl;
		}
	}

	void FEMIntegrator::Stretch()
	{
	/*	for(int i=0; i<m_controledNode.size(); i++)
		{
			int idx = m_controledNode[i];

			Vector3d force = Vector3d(0, 0, 1.0);
			m_externalForce[idx] = m_externalForce[idx] + force * 100.0;

		}*/

		//Vector3d force = Vector3d(-10, -5, 1.0);
		//m_externalForce[25] = m_externalForce[25] + force;

		//force = Vector3d(0, 0, -1.0);
		//m_externalForce[627] = m_externalForce[627] + force*500;

		//force = Vector3d(0, 0, -1.0);
		//m_externalForce[613] = m_externalForce[613] + force*500;

		//force = Vector3d(0, 1, 0);
		//m_externalForce[949] = m_externalForce[949] + force;

		//force = Vector3d(1, 0.5, 0.0);
		//m_externalForce[182] = m_externalForce[182] + force;

	/*	Vector3d force = Vector3d(0, -3, 0.0);
		m_externalForce[717] = m_externalForce[182] + force;
		m_externalForce[222] = m_externalForce[182] + force;*/
	}

	void FEMIntegrator::ComputeEigenMode(int numModes, std::vector<double>& _freq, std::vector<std::vector<Vector3d>>& _modes)
	{
		if (m_flagNeedReInitInternalSolver)
		{
			VegaFEMInterface::Instance(m_integratorId)->Initialize(
				m_pModel->GetTopology(),
				m_vertMass,
				m_constrainedDof);
			m_flagNeedReInitInternalSolver = false;
		}

		std::vector<double> tmpMode(numModes*_modes[0].size()*3);
		VegaFEMInterface::Instance(m_integratorId)->ComputeLinearMode(numModes, GetModel()->GetElementUndeformedStiffnessMatrix(), _freq, tmpMode, m_vertMass);

		for(int i=0; i<numModes; i++)
		{
			for(int j=0; j<_modes[0].size(); j++)
			{
				_modes[i][j].X() = tmpMode[i*_modes[0].size()*3+j*3] * m_vertMass[j];
				_modes[i][j].Y() = tmpMode[i*_modes[0].size()*3+j*3+1] * m_vertMass[j];
				_modes[i][j].Z() = tmpMode[i*_modes[0].size()*3+j*3+2] * m_vertMass[j];
			}
		}
	}

	void FEMIntegrator::ComputeEigenMode(int numModes, int numSketchData, double* sketchData, std::vector<double>& _freq, std::vector<std::vector<Vector3d>>& _modes)
	{
		if (m_flagNeedReInitInternalSolver)
		{
			VegaFEMInterface::Instance(m_integratorId)->Initialize(
				m_pModel->GetTopology(),
				m_vertMass,
				m_constrainedDof);
			m_flagNeedReInitInternalSolver = false;
		}

		std::vector<double> tmpMode(numModes*_modes[0].size()*3);
		VegaFEMInterface::Instance(m_integratorId)->ComputeNonLinearMode(numModes, _freq, tmpMode, m_vertMass, sketchData, numSketchData);

		for(int i=0; i<numModes; i++)
		{
			for(int j=0; j<_modes[0].size(); j++)
			{
				_modes[i][j].X() = tmpMode[i*_modes[0].size()*3+j*3];
				_modes[i][j].Y() = tmpMode[i*_modes[0].size()*3+j*3+1];
				_modes[i][j].Z() = tmpMode[i*_modes[0].size()*3+j*3+2];
			}
		}
	}


	bool FEMIntegrator::BackwardEulerStepContact(double t)
	{
		m_nodalPositionLast = m_nodalPosition;
		m_nodalVelocityLast = m_nodalVelocity;

		size_t cdof = m_constrainedDof.size();

		double *pos = reinterpret_cast<double *>(m_nodalPosition.data());
		double *vel = reinterpret_cast<double *>(m_nodalVelocity.data());
		for (size_t ci = 0; ci < cdof; ++ci)
		{
			m_constrainedDofPosBuf[ci] = pos[m_constrainedDof[ci]];
			m_constrainedDofVelBuf[ci] = vel[m_constrainedDof[ci]];
		}


		if (m_flagNeedReInitInternalSolver)
		{
			VegaFEMInterface::Instance(m_integratorId)->Initialize(
				m_pModel->GetTopology(),
				m_vertMass,
				m_constrainedDof);
			m_flagNeedReInitInternalSolver = false;
		}

		m_pModel->ResetAllFlags();
		m_pModel->UpdateForceStiffnessMatrix(m_nodalPosition);
		m_pModel->ComputeElasticForce(m_nodalPosition, m_nodalForce, CorotatedLinearModel::StiffneeMatrix);
		auto &elmStiffness = m_pModel->GetElementStiffnessMatrix();

		for (size_t i = 0; i < m_addedForceVert.size(); ++i)
			m_externalForce[m_addedForceVert[i]] = m_addedForces[i];

		VegaFEMInterface::StepTask task = {            
			m_nodalPosition,        // std::vector<Vector3d> &nodalPosition;
			m_nodalVelocity,        // std::vector<Vector3d> &nodalVelocity;
			m_nodalForce,           // const std::vector<Vector3d> &nodalForce;
			m_externalForce,        // const std::vector<Vector3d> &externalForce;
			m_zeroForce,            // const Vector3d &g;
			elmStiffness,           // const std::vector<std::array<double, 12*12>> &elmStiffness;
			m_dampingStiffnessCoef, // double dampingStiffnessCoef;
			m_dampingMassCoef       // double dampingMassCoef;
		};
		bool flag = VegaFEMInterface::Instance(m_integratorId)->DoTimeStep(t, task);

		for (size_t ci = 0; ci < cdof; ++ci)
		{
			pos[m_constrainedDof[ci]] = m_constrainedDofPosBuf[ci];
			vel[m_constrainedDof[ci]] = m_constrainedDofVelBuf[ci];
		}

		return flag;

	}

	bool FEMIntegrator::BackwardEulerStepRestShape(double t)
	{
		m_nodalPositionLast = m_nodalPosition;
		m_nodalVelocityLast = m_nodalVelocity;

		size_t cdof = m_constrainedDof.size();

		double *pos = reinterpret_cast<double *>(m_nodalPosition.data());
		double *vel = reinterpret_cast<double *>(m_nodalVelocity.data());
		for (size_t ci = 0; ci < cdof; ++ci)
		{
			m_constrainedDofPosBuf[ci] = pos[m_constrainedDof[ci]];
			m_constrainedDofVelBuf[ci] = vel[m_constrainedDof[ci]];
		}

		if (m_flagNeedReInitInternalSolver)
		{
			VegaFEMInterface::Instance(m_integratorId)->Initialize(
				m_pModel->GetTopology(),
				m_vertMass,
				m_constrainedDof);
			m_flagNeedReInitInternalSolver = false;
		}

		// update parameter

		//m_restShapeProblem->UpdateTotalNodalForceAndStiffness(m_nodalPosition);
		m_restShapeProblem->RestShapeProblem::Update(m_nodalPosition, 
			(RestShapeProblem::UT_NodalForce | RestShapeProblem::UT_StiffnessMatrix));

		auto &exampleForce = m_restShapeProblem->GetExampleForce();
		auto &nodalForce   = m_restShapeProblem->GetNodalForce();
		auto &elmStiffness = m_restShapeProblem->GetElmMaterialStiffnessMatrix();

		size_t nvert = m_nodalPosition.size();
		for (size_t vi = 0; vi < nvert; ++vi)
			m_nodalForce[vi] = exampleForce[vi] - nodalForce[vi];

		VegaFEMInterface::StepTask task = {            
			m_nodalPosition,        // std::vector<Vector3d> &nodalPosition;
			m_nodalVelocity,        // std::vector<Vector3d> &nodalVelocity;
			nodalForce,             // const std::vector<Vector3d> &nodalForce;
			exampleForce,            // const std::vector<Vector3d> &externalForce;
			m_zeroForce,            // const Vector3d &g;
			elmStiffness,           // const std::vector<std::array<double, 12*12>> &elmStiffness;
			m_dampingStiffnessCoef, // double dampingStiffnessCoef;
			m_dampingMassCoef       // double dampingMassCoef;
		};
		bool flag = VegaFEMInterface::Instance(m_integratorId)->DoTimeStep(t, task);

		for (size_t ci = 0; ci < cdof; ++ci)
		{
			pos[m_constrainedDof[ci]] = m_constrainedDofPosBuf[ci];
			vel[m_constrainedDof[ci]] = m_constrainedDofVelBuf[ci];
		}
		return flag;
	}



	void FEMIntegrator::AddForce(const std::vector<int> &vert, const std::vector<Vector3d> &forces)
	{
		m_addedForceVert.insert(m_addedForceVert.end(), vert.begin(), vert.end());
		m_addedForces.insert(m_addedForces.end(), forces.begin(), forces.end());
	}

	double FEMIntegrator::ComputeTotalForceResidual(void)
	{
		double change = 0.0;
		for (size_t i = 0; i < m_nodalForce.size(); ++i)
			change += m_nodalForce[i].SquaredLength();

		change = sqrt(change) / (double) m_nodalForce.size();

		return change;
	}

	double FEMIntegrator::ComputeMaxForceResidual(void)
	{
		std::vector<double> node_force_res;
		node_force_res.resize(m_nodalForce.size(), 0.0);

		for(size_t i=0; i<m_nodalForce.size(); i++)
			node_force_res[i] = m_nodalForce[i].Normalize();

		size_t cdof = m_constrainedDof.size();
		for (size_t ci = 0; ci < cdof; ++ci)
			node_force_res[ci] = 0.0;

		double change = 0.0;
		for(size_t i=0; i<node_force_res.size(); i++)
		{
			if(node_force_res[i]>change)
				change = node_force_res[i];
		}

		return change;
	}

	void FEMIntegrator::InitAnimation(const QString cofigPath)
	{
		/*m_pAnimation = new Animation();
		m_pAnimation->LoadBody(cofigPath);
		m_pAnimation->LoadFoot(cofigPath);
*/

	/*	m_pDinosaruAnimation = new DinosaruAnimation();
		m_pDinosaruAnimation->LoadRefShape(cofigPath);
		m_pDinosaruAnimation->LoadCtrlPoint(cofigPath);
		m_pDinosaruAnimation->LoadAnimData(cofigPath);
		m_pDinosaruAnimation->LoadWeight(cofigPath);
		m_pDinosaruAnimation->Process_Data();*/

	/*	m_pGreenAnimation = new GreenRubberAnimation();
		m_pGreenAnimation->LoadRefShape(cofigPath);
		m_pGreenAnimation->LoadCtrlPoint(cofigPath);
		m_pGreenAnimation->LoadWeight(cofigPath);*/

	/*	m_pHangerAnimation = new HangerAnimation();
		m_pHangerAnimation->LoadRefShape(cofigPath);
		m_pHangerAnimation->LoadCtrlPoint(cofigPath);
		m_pHangerAnimation->LoadAnimData(cofigPath);
		m_pHangerAnimation->LoadWeight(cofigPath);
		m_pHangerAnimation->Process_Data();*/

	}

}