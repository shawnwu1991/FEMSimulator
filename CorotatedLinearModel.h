#pragma once

#include "TetMesh.h"
#include "VecMatInterface.h"

#include <vector>

namespace Libin
{

    class CorotatedLinearModel
    {
    private:
        CorotatedLinearModel(const CorotatedLinearModel &);
        CorotatedLinearModel &operator = (const CorotatedLinearModel &);
    public:
        CorotatedLinearModel(std::shared_ptr<TetMeshTopology> topology);
        ~CorotatedLinearModel(void);

    public:
        enum ForceComputationMethod { StressTensor, StiffneeMatrix };

    public:
        // tetMesh
        //TetMesh *GetTetMesh(void) const { return m_pTetMesh; }
        std::shared_ptr<TetMeshTopology> GetTopology(void) const { return m_topology; }
        // initialization
        void InitializeFirstPiolaKirchhoffMethod(TetMesh *pTetMesh);
        void InitializeUndeformedStiffnessMatrix(TetMesh *pTetMesh);

        // all node operator
        void ComputeElasticForce(const std::vector<Vector3d> &nodalPosition, std::vector<Vector3d> &nodalForce, ForceComputationMethod method = StressTensor);
        void UpdateForceStiffnessMatrix(const std::vector<Vector3d> &nodalPosition);

        void UpdateForceMaterialStiffnessMatrix(const std::vector<Vector3d> &nodalPosition);
        void UpdateLambdaAlphaDerivative(const std::vector<Vector3d> &nodalPosition);

        const std::vector<std::array<double, 12*12>> &GetElementStiffnessMatrix(void) const { return m_elmStiffness; }
        const std::vector<std::array<double, 12*12>> &GetElementMaterialStiffnessMatrix(void) const { return m_elmMaterialStiffness; }
		const std::vector<std::array<double, 12*12>> &GetElementUndeformedStiffnessMatrix(void) const { return m_elmUndeformedStiffness; }

        const std::vector<std::array<double, 12*1>> &GetElementLambdaDerivative(void) { return m_elmDfDlambda; }
        const std::vector<std::array<double, 12*1>> &GetElementAlphaDerivative(void) { return m_elmDfDalpha; }


        // per node operator
        Vector3d ComputeElasticForceForNode(int nodeId, const std::vector<Vector3d> &nodalPosition, ForceComputationMethod method = StressTensor, bool reCalc = false);
        Matrix3d ComputeStiffnessMatrixForNode(int nodeId, const std::vector<Vector3d> &nodalPosition, bool reCalc = false);
        Matrix3d ComputeMaterialStiffnessMatrixForNode(int nodeId, const std::vector<Vector3d> &nodalPosition, bool reCalc = false);

        // 
        bool ComputeMaterialStiffnessMatrixFor(int fid, int nid, Matrix3d &stiff, bool reCalc = false);
        bool ComputeStiffnessMatrixFor(int fid, int nid, Matrix3d &stiff, bool reCalc = false);

        // per element operator
        const std::array<Vector3d, 4> &ComputeElasticForceForElement(size_t elmId, const std::vector<Vector3d> &nodalPosition, ForceComputationMethod method = StressTensor, bool reCalc = false);
        const std::array<Vector3d, 4> &ComputeElasticForceForElementUsingStressTensor(size_t elmId, const std::vector<Vector3d> &nodalPosition, bool reCalc = false);
        const std::array<Vector3d, 4> &ComputeElasticForceForElementUsingStiffnessMatrix(size_t elmId, const std::vector<Vector3d> &nodalPosition, bool reCalc = false);

        const std::array<double, 12*12> &ComputeStiffnessMatrixForElement(size_t elmId, const std::vector<Vector3d> &nodalPosition, bool reCalc = false);
        const std::array<double, 12*12> &ComputeMaterialStiffnessMatrixForElement(size_t elmId, const std::vector<Vector3d> &nodalPosition, bool reCalc = false);

        // flags
        void ResetStiffnessFlags(void);
        void ResetMaterialStiffnessFlags(void);
        void ResetFRSFlags(void);
        void ResetForceFlags(void);
        void ResetLambdaAlphaDerivativeFlags(void);
        void ResetAllFlags(void);

    protected:
        void UpdateStiffnessMatrixForElement(size_t elmId, const std::vector<Vector3d> &nodalPosition);
        void UpdateMaterialStiffnessMatrixForElement(size_t elmId, const std::vector<Vector3d> &nodalPosition);
        void UpdateLambdaAlphaDerivativeForElement(size_t elmId, const std::vector<Vector3d> &nodalPosition);
        void UpdateElasticForceForElementUsingStressTensor(size_t elmId, const std::vector<Vector3d> &nodalPosition);
        void UpdateElasticForceForElementUsingStiffnessMatrix(size_t elmId, const std::vector<Vector3d> &nodalPosition);

        typedef void (CorotatedLinearModel::*PerElementOperator)(size_t, const std::vector<Vector3d> &);
        virtual void UpdateAllElements(const std::vector<Vector3d> &nodalPosition, PerElementOperator fns[], int n);

    private:
        //TetMesh *m_pTetMesh;
        std::shared_ptr<TetMeshTopology> m_topology;

        std::vector<Matrix3d> m_elmF;
        std::vector<Matrix3d> m_elmR;
        std::vector<Matrix3d> m_elmS;
        std::vector<bool>     m_flagFRSCalculated;
        
        // use first Piola-Kirchhoff stress tensor to compute force
        // this way is faster than computing with stiffness matrix 
        // Dm, Dm^-1 for each element
        std::vector<Matrix3d> m_elmDs;
        std::vector<Matrix3d> m_elmDm;
        std::vector<Matrix3d> m_elmBm;  // Bm=Dm^-1
        std::vector<Matrix3d> m_elmWBmT;   // w * Bm

        std::vector<double> m_elmVolume;
        std::vector<double> m_elmStiffMu;
        std::vector<double> m_elmStiffLambda;
    
        // per element stiffness matrix
        // each stiffness matrix is a 12x12 matrix
        std::vector<std::array<double, 12*12>> m_elmUndeformedStiffness;
        std::vector<std::array<double, 12*12>> m_elmStiffness;
        std::vector<bool> m_flagElmStifnessCalculated;
        
        std::vector<std::array<double, 12*12>> m_elmBtGB;
        std::vector<std::array<double, 12*12>> m_elmBtHB;
        std::vector<std::array<double, 12*1>> m_elmDfDlambda;
        std::vector<std::array<double, 12*1>> m_elmDfDalpha;
        std::vector<bool> m_flagDfDlambdaDalphaCalculated;

        std::vector<Matrix4d> m_elmVs;
        std::vector<Matrix4d> m_elmVm;
        std::vector<Matrix4d> m_elmVBm;
        std::vector<std::array<double, 12*12>> m_elmVolBtE;

        std::vector<std::array<double, 12*12>> m_elmMaterialStiffness;
        std::vector<bool> m_flagElmMaterialStiffnessCalculated;

        std::vector<std::array<Vector3d, 4>> m_elmElasticForces;
        std::vector<bool> m_flagElmElasticForceCalculated;
    };

}