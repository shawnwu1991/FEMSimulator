#pragma once

#include "VecMatInterface.h"
#include "StdStrings.h"

#include "TetMesh.h"
#include "CorotatedLinearModel.h"

#include <vector>

namespace Libin
{

	class RestShapeProblem
	{
	private:
		RestShapeProblem(const RestShapeProblem &);
		RestShapeProblem &operator =(const RestShapeProblem &);

	public:
		RestShapeProblem(std::shared_ptr<TetMeshTopology> topology);
		virtual ~RestShapeProblem(void);

		bool LoadStaticFrame(const tstring &frame, int numLineToSkip, std::shared_ptr<TetMesh>& staticFrame);
		virtual bool Initialize(const Vector3d &gravity, const std::vector<double> &vertMass,  const std::shared_ptr<TetMesh>& tetMesh);
		virtual void Update(const std::vector<Vector3d> &restShape, int type, int start = 1, int end = -1);
		
		std::shared_ptr<TetMesh> GetStaticShape() const { return m_staticShape; }
		std::shared_ptr<TetMeshMaterials> GetMaterial(void) const { return m_materials; }
		const std::vector<std::array<double, 12*12>> &GetElmMaterialStiffnessMatrix(void) const { return m_totalElmMaterialStiffness; }
		const std::vector<Vector3d> &GetExampleForce(void) const { return m_staticExampleForce; }
		const std::vector<Vector3d> &GetNodalForce(void) const { return m_staticNodalForce; }

		void SetUniformMaterial(double E, double nu, double density) { m_materials->SetUniformMaterial(E, nu, density, m_topology->GetNumElement()); }
		void SetUniformMaterial(const TetMeshMaterials::Material &mt) { m_materials->SetUniformMaterial(mt, m_topology->GetNumElement()); }
		void SetMaterial(const TetMeshMaterials* mtPtr);

		enum UpdateType {
			UT_NodalForce = 1, 
			UT_StiffnessMatrix = 2, 
			UT_VolumePreservation = 5,
			UT_All = UT_NodalForce | UT_StiffnessMatrix 
		};

	protected:
		void ComputeExampleForce(const Vector3d &gravity, const std::vector<double> &vertMass);

	public:
		int    m_numElement;
		int    m_numNode;

		std::shared_ptr<TetMeshTopology> m_topology;
		std::shared_ptr<TetMeshMaterials> m_materials;

	public:
		std::shared_ptr<TetMesh> m_staticShape;
		std::shared_ptr<CorotatedLinearModel> m_model;

		std::vector<std::array<double, 12*12>> m_totalElmMaterialStiffness;

		std::vector<Vector3d> m_staticNodalForce;
		std::vector<Vector3d> m_staticExampleForce;
		std::vector<Vector3d> m_staticForceRes;
	};

}
