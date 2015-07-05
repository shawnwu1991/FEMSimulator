#pragma once

#include "..\VegaFEM\sparseMatrix\sparseMatrix.h"
#include "..\VegaFEM\sparseSolver\CGSolver.h"
//#include "..\VegaFEM\sparseSolver\PardisoSolver.h"
#include "..\\VegaFEM\sparseSolver\ARPACKSolver.h"
#include "mkl.h"

#include <vector>
#include <memory>

#include "TetMesh.h"

namespace Libin
{
    class VegaFEMInterface
    {
    private:
        VegaFEMInterface(void);

    public:
        ~VegaFEMInterface(void);
        static std::shared_ptr<VegaFEMInterface> Instance(int id);

    public:
        void Initialize(std::shared_ptr<TetMeshTopology> topology, const std::vector<double> &lumpedMass, const std::vector<int> &constrainedDOFs);
        void AssembleStiffnessMatrix(const std::vector<std::array<double, 12*12>> &elmStiffness);
		void AssembleUndeformedStiffnessMatrix(const std::vector<std::array<double, 12*12>> &elmStiffness);
		void ComputeLinearMode(int numDesiredMode, const std::vector<std::array<double, 12*12>> &elmUndeformedStiffness, std::vector<double>& _freq, 
			std::vector<double>& _modes, const std::vector<double> &lumpedMass);
		void ComputeNonLinearMode(int numDesiredMode, std::vector<double>& _freq, std::vector<double>& _modes, const std::vector<double> &lumpedMass,
			double* sketchData, int numSketchFrame);
        
		struct StepTask
        {
            std::vector<Vector3d> &nodalPosition;
            std::vector<Vector3d> &nodalVelocity;
            const std::vector<Vector3d> &nodalForce;
            const std::vector<Vector3d> &externalForce;
            const std::vector<Vector3d> &gravityForce;
            const std::vector<std::array<double, 12*12>> &elmStiffness;
            double dampingStiffnessCoef;
            double dampingMassCoef;
        };
        bool DoTimeStep(double t, StepTask &task);

    private:
        std::shared_ptr<TetMeshTopology> m_tetTopology;
        std::shared_ptr<SparseMatrix> m_massMatrix; 
        std::shared_ptr<SparseMatrix> m_rayleighDampingMatrix;
        std::shared_ptr<SparseMatrix> m_tangentStiffnessMatrix;
		std::shared_ptr<SparseMatrix> m_undeformedStiffnessMatrix;
        std::shared_ptr<SparseMatrix> m_systemMatrix;
        std::shared_ptr<CGSolver> m_CGSolver;
        std::vector<double> m_bufferConstrained;
        std::vector<double> m_buffer;
        std::vector<double> m_qresidual;
        std::vector<double> m_qdelta;
        std::vector<int> m_constrainedDOFs;
        std::vector<std::array<int, 16>> m_columnIndices;
        int m_dof;

        //std::shared_ptr<PardisoSolver> m_PardisoSolverSolver;
        struct PardisoSolverParam
        {
            int n;
            int numThreads;
            int positiveDefinite;
            int directIterative;
            int verbose;
            double * a;
            int *perm;
            int * ia, * ja;

            void *pt[64];
            MKL_INT iparm[64];
            int mtype;
            MKL_INT nrhs; 
            MKL_INT maxfct, mnum, phase, error, msglvl;
        };
        PardisoSolverParam m_pardParam;

    private:
        static std::map<int, std::shared_ptr<VegaFEMInterface>> ms_instances;
    };
}