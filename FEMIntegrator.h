#pragma once

#include "AbstractIntegrator.h"
#include <memory>
#include "Animation.h"
#include "DinosaruAnimation.h"
#include "GreenRubberAnimation.h"
#include "HangerAnimation.h"

namespace Libin
{
    class CorotatedLinearModel;
    class TetMesh;
	class RestShapeProblem;

    class FEMIntegrator : public AbstractIntegrator
    {
    public:
        FEMIntegrator(CorotatedLinearModel *pModel, TetMesh *tetMesh, std::shared_ptr<RestShapeProblem> restShapeProblem = std::shared_ptr<RestShapeProblem>());
        ~FEMIntegrator(void);

    public:
        void SetGravity(const Vector3d &g);
        void SetDampingParameters(double dampingStiffnessCoef, double dampingMassCoef);
        void SetState(const std::vector<Vector3d> &nodalPosition, const std::vector<Vector3d> &nodalVelocity = std::vector<Vector3d>());
        
        void SymplecticEulerStep(double t);
        bool BackwardEulerStep(double t);
		bool BackwardEulerStepRestShape(double t);
		bool BackwardEulerStepContact(double t);

        const std::vector<Vector3d> &GetNodalPosition(void) const { return m_nodalPosition; }
        const std::vector<Vector3d> &GetNodalVelocity(void) const { return m_nodalVelocity; }
        const std::vector<Vector3d> &GetLastStepNodalForce(void) const { return m_nodalForce; }
		const std::vector<Vector3d> &GetLastNodalPosition(void) const {return m_nodalPositionLast; }
		const std::vector<Vector3d> &GetExternalForce(void) const { return m_externalForce;}
		CorotatedLinearModel* GetModel() { return m_pModel; }

        void ClearInternalForce(void);
        void ClearExternalForce(void);

        void AddExternalForce(const std::vector<Vector3d> &forces);
        void AddForce(const std::vector<int> &vert, const std::vector<Vector3d> &forces);

        void ConstrainNodesInRange(double range[6], TetMesh *tetMesh);
        void ConstrainNodesByIndex(std::vector<int> nodeIdx);
		void ControledNodesByIndex(std::vector<int> nodeIdx);

		double ComputeTotalForceResidual(void);
		double ComputeMaxForceResidual(void);

		void ConstraintEndPoint(double t);
		void RotateZ();
		void Drag();
		void Stretch();
		void ComputeEigenMode(int numModes, std::vector<double>& _freq, std::vector<std::vector<Vector3d>>& _modes);
		void ComputeEigenMode(int numModes, int numSketchData, double* sketchData, std::vector<double>& _freq, std::vector<std::vector<Vector3d>>& _modes);

		double GetMassDamping(){return m_dampingMassCoef; }
		double GetStiffnessDamping(){return m_dampingStiffnessCoef; }

		void InitAnimation(const QString cofigPath);

    private:
        CorotatedLinearModel *m_pModel;
        std::vector<double> m_vertMass;
		TetMesh              *m_pTetMesh;
		std::shared_ptr<RestShapeProblem> m_restShapeProblem;
		Animation *m_pAnimation;
		DinosaruAnimation *m_pDinosaruAnimation;
		GreenRubberAnimation *m_pGreenAnimation;
		HangerAnimation *m_pHangerAnimation;
		

        int m_integratorId;
        bool m_flagNeedReInitInternalSolver;

        double m_dampingStiffnessCoef;
        double m_dampingMassCoef;
        
        std::vector<Vector3d> m_nodalPositionLast;
        std::vector<Vector3d> m_nodalVelocityLast;

        std::vector<Vector3d> m_nodalPosition;
        std::vector<Vector3d> m_nodalVelocity;
        std::vector<Vector3d> m_nodalForce;

        std::vector<Vector3d> m_externalForce;
        std::vector<Vector3d> m_gravityForce;
		std::vector<Vector3d> m_airForce;
        std::vector<Vector3d> m_zeroForce;

        Vector3d m_gravity;
        std::vector<int> m_constrainedDof;
        std::vector<double> m_constrainedDofPosBuf;
        std::vector<double> m_constrainedDofVelBuf;

		std::vector<int> m_controledNode;

        std::vector<int> m_addedForceVert;
        std::vector<Vector3d> m_addedForces;
    };


}

