#include "stdafx.h"
#include "VegaFEMInterface.h"
#include "insertRows\insertRows.h"
#include "matrix\matrixMacros.h"
#include "matrix\matrixPCA.h"

//#include "cv.h"

#pragma comment (lib, "VegaFEM.lib")
//#if _DEBUG
//#pragma comment (lib, "opencv_core248d.lib")
//#else
//#pragma comment (lib, "opencv_core248.lib")
//#endif


namespace Libin
{
    std::map<int, std::shared_ptr<VegaFEMInterface>> VegaFEMInterface::ms_instances;

    VegaFEMInterface::VegaFEMInterface(void)
        : m_dof(0)
    {
        m_pardParam.a  = NULL;
        m_pardParam.ia = NULL;
        m_pardParam.ja = NULL;
        m_pardParam.perm = NULL;
    }

    VegaFEMInterface::~VegaFEMInterface(void)
    {        
        free(m_pardParam.a);
        free(m_pardParam.ia);
        free(m_pardParam.ja);
        free(m_pardParam.perm);
         
        m_pardParam.phase = -1;
        PARDISO(
            m_pardParam.pt,
            &m_pardParam.maxfct,
            &m_pardParam.mnum,
            &m_pardParam.mtype,
            &m_pardParam.phase,
            &m_pardParam.n,
            m_pardParam.a,
            m_pardParam.ia, 
            m_pardParam.ja,
            NULL,
            &m_pardParam.nrhs,
            m_pardParam.iparm,
            &m_pardParam.msglvl,
            NULL,
            NULL,
            &m_pardParam.error
            );
        
        if (m_pardParam.error != 0)
            printf("Error: Pardiso dealloacation returned non-zero exit code %d.\n", m_pardParam.error);

    }

    std::shared_ptr<VegaFEMInterface> VegaFEMInterface::Instance(int id)
    {
        auto &inst = ms_instances[id];
        if (!inst)
            inst.reset(new VegaFEMInterface);
        return inst;
    }
    
    void VegaFEMInterface::Initialize(std::shared_ptr<TetMeshTopology> topology, 
        const std::vector<double> &lumpedMass, const std::vector<int> &constrainedDOFs)
    {
        m_tetTopology = topology;
        int nvert = topology->GetNumVertices();
        int nelm = (int) topology->GetElements().size();
        m_dof = nvert * 3;
        m_constrainedDOFs = constrainedDOFs;

        // mass matrix
        std::shared_ptr<SparseMatrixOutline> massMatrixOutline = std::make_shared<SparseMatrixOutline>(m_dof);
        for (int vi = 0; vi < nvert; ++vi)
            massMatrixOutline->AddBlock3x3DiagEntry(vi, vi, lumpedMass[vi]);
        m_massMatrix = std::make_shared<SparseMatrix>(massMatrixOutline.get());

        // stiffness matrix        
        std::shared_ptr<SparseMatrixOutline> stiffnessMatrixOutline = std::make_shared<SparseMatrixOutline>(m_dof);
        for (int el = 0; el < nelm; ++el)
        {
            auto &vtxIndex = topology->GetElements()[el];

            for (int i=0; i<4; i++)
                for (int j=0; j<4; j++)
                {
                    // add 3x3 block corresponding to pair of vertices (i,j)
                    stiffnessMatrixOutline->AddBlock3x3Entry(vtxIndex[i], vtxIndex[j], 0.0);
                }
        }

        m_tangentStiffnessMatrix = std::make_shared<SparseMatrix>(stiffnessMatrixOutline.get());

		// undeformedstiffness matrix        
        std::shared_ptr<SparseMatrixOutline> undeformedstiffnessMatrixOutline = std::make_shared<SparseMatrixOutline>(m_dof);
        for (int el = 0; el < nelm; ++el)
        {
            auto &vtxIndex = topology->GetElements()[el];

            for (int i=0; i<4; i++)
                for (int j=0; j<4; j++)
                {
                    // add 3x3 block corresponding to pair of vertices (i,j)
                    undeformedstiffnessMatrixOutline->AddBlock3x3Entry(vtxIndex[i], vtxIndex[j], 0.0);
                }
        }

		m_undeformedStiffnessMatrix = std::make_shared<SparseMatrix>(undeformedstiffnessMatrixOutline.get());
        
        m_rayleighDampingMatrix = std::make_shared<SparseMatrix>(*m_tangentStiffnessMatrix);
        m_rayleighDampingMatrix->BuildSubMatrixIndices(*m_massMatrix);
        m_tangentStiffnessMatrix->BuildSubMatrixIndices(*m_massMatrix);

        // initialize system matrix and solver        
        m_systemMatrix = std::make_shared<SparseMatrix>(*m_tangentStiffnessMatrix);
        m_systemMatrix->RemoveRowsColumns((int) m_constrainedDOFs.size(), m_constrainedDOFs.data());
        m_systemMatrix->BuildSuperMatrixIndices((int) m_constrainedDOFs.size(), m_constrainedDOFs.data(), m_tangentStiffnessMatrix.get());        

        m_CGSolver = std::make_shared<CGSolver>(m_systemMatrix.get());
        //m_PardisoSolverSolver.reset(new PardisoSolver(m_systemMatrix.get(), 16));

        //  indices for assembling stiffness matrix
        auto &elements = topology->GetElements();
        m_columnIndices.resize(nelm);
        for (int el = 0; el < nelm; el++)
        {
            // the 4 rows corresponding to the 4 vertices
            auto &rowIndices = elements[el];
            // the 4 columns corresponding to all 4 vertices, in row of each vertex
            // find index of vertex j in row of vertex i, and cache it
            auto &columnIndices = m_columnIndices[el];
            for(int i=0; i<4; i++)
                for(int j=0; j<4; j++)
                    columnIndices[4 * i + j] = m_tangentStiffnessMatrix->GetInverseIndex(3 * rowIndices[i], 3 * rowIndices[j]) / 3;
        }

        // initialize buffers
        m_buffer.resize(m_dof, 0.0);
        m_bufferConstrained.resize(m_dof - (int) m_constrainedDOFs.size());
        m_qresidual.resize(m_dof, 0.0);
        m_qdelta.resize(m_dof, 0.0);


        // pardsolver
        m_pardParam.numThreads = 16;
        m_pardParam.positiveDefinite = 0;
        m_pardParam.directIterative = 0;
        m_pardParam.verbose = 0;
        m_pardParam.n = m_systemMatrix->Getn();
        int nnz = m_systemMatrix->GetNumEntries();
        m_pardParam.a = (double*) malloc (sizeof(double) * nnz);  
        m_pardParam.ia = (int*) malloc (sizeof(int) * (m_systemMatrix->GetNumRows() + 1));  
        m_pardParam.ja = (int*) malloc (sizeof(int) * nnz);  
        m_pardParam.perm = (int*) malloc (sizeof(int) * m_pardParam.n);
        memset(m_pardParam.perm, 0, sizeof(int) * m_pardParam.n);

        m_systemMatrix->GenerateCompressedRowMajorFormat(
            m_pardParam.a, m_pardParam.ia, m_pardParam.ja, 0, 1);

        m_pardParam.mtype = 11; // real and nonsymmetric
        m_pardParam.nrhs = 1; // Number of right-hand sides that need to be solved for.
        m_pardParam.maxfct = 1; // Maximum number of numerical factorizations.
        m_pardParam.mnum = 1;   // Which factorization to use;
        m_pardParam.msglvl = 0;   // Print statistical information in file;
        m_pardParam.error = 0;

        for (int i = 0; i < 64; i++) 
            m_pardParam.iparm[i] = 0;
        m_pardParam.iparm[ 1 - 1] = 1;// 0; use defaut 1; // No solver default
        m_pardParam.iparm[ 2 - 1] = 2; // 0=minimum degree ordering, 2=Fill-in reordering from METIS
        m_pardParam.iparm[ 3 - 1] = 0; //m_pardParam.numThreads; // Numbers of processors, value of OMP_NUM_THREADS
        m_pardParam.iparm[ 4 - 1] = 0; //m_pardParam.directIterative ? 62 : 0; //62; // No iterative-direct algorithm
        m_pardParam.iparm[ 5 - 1] = 0; // No user fill-in reducing permutation
        m_pardParam.iparm[ 6 - 1] = 0; // Write solution into x
        m_pardParam.iparm[ 7 - 1] = 0; // Not in use
        m_pardParam.iparm[ 8 - 1] = 0; // Max numbers of iterative refinement steps
        m_pardParam.iparm[ 9 - 1] = 0; // Not in use
        m_pardParam.iparm[10 - 1] = 13; // Perturb the pivot elements with 1E-13
        m_pardParam.iparm[11 - 1] = 1; // Use nonsymmetric permutation and scaling MPS
        m_pardParam.iparm[12 - 1] = 0; // Not in use
        m_pardParam.iparm[13 - 1] = 0; // matchings for highly indefinite symmetric matrices
        m_pardParam.iparm[14 - 1] = 0; // Output: Number of perturbed pivots
        m_pardParam.iparm[15 - 1] = 0; // Not in use
        m_pardParam.iparm[16 - 1] = 0; // Not in use
        m_pardParam.iparm[17 - 1] = 0; // Not in use
        m_pardParam.iparm[18 - 1] = -1; // Output: Number of nonzeros in the factor LU
        m_pardParam.iparm[19 - 1] = 0; // no Output: Mflops for LU factorization
        m_pardParam.iparm[20 - 1] = 0; // Output: Numbers of CG Iterations
        m_pardParam.iparm[21 - 1] = 1; // pivoting method
         
        for (int i=0; i<64; i++) 
            m_pardParam.pt[i] = 0;
        
        //m_pardParam.msglvl = 1;
        m_pardParam.phase = 11;
        pardiso(
            m_pardParam.pt, 
            &m_pardParam.maxfct, 
            &m_pardParam.mnum, 
            &m_pardParam.mtype,
            &m_pardParam.phase,
            &m_pardParam.n, 
            m_pardParam.a, 
            m_pardParam.ia, 
            m_pardParam.ja, 
            m_pardParam.perm, 
            &m_pardParam.nrhs,
            m_pardParam.iparm,
            &m_pardParam.msglvl, 
            NULL, 
            NULL, 
            &m_pardParam.error
            );
      
        if (m_pardParam.error != 0)
        {
            printf("Error: Pardiso matrix re-ordering returned non-zero exit code %d.\n", m_pardParam.error);
        }
    }
       
    void VegaFEMInterface::AssembleStiffnessMatrix(const std::vector<std::array<double, 12*12>> &elmStiffness)
    {
        auto &elements = m_tetTopology->GetElements();
        size_t nelm = elements.size();

        auto &stiffnessMatrix = *m_tangentStiffnessMatrix;
        stiffnessMatrix.ResetToZero();

        for (size_t el = 0; el < elements.size(); ++el)
        {
            auto &rowIndex = elements[el];
            auto &columnIndex = m_columnIndices[el];
            const double *KElement = elmStiffness[el].data();

            // add KElement to the global stiffness matrix
            for (int i=0; i<4; i++)
                for (int j=0; j<4; j++)
                    for(int k=0; k<3; k++)
                        for(int l=0; l<3; l++)
                            stiffnessMatrix.AddEntry(3 * rowIndex[i] + k, 3 * columnIndex[4 * i + j] + l, KElement[12 * (3 * i + k) + 3 * j + l]);
        }
    }


	void VegaFEMInterface::AssembleUndeformedStiffnessMatrix(const std::vector<std::array<double, 12*12>> &elmStiffness)
    {
        auto &elements = m_tetTopology->GetElements();
        size_t nelm = elements.size();

        auto &stiffnessMatrix = *m_undeformedStiffnessMatrix;
        stiffnessMatrix.ResetToZero();

        for (size_t el = 0; el < elements.size(); ++el)
        {
            auto &rowIndex = elements[el];
            auto &columnIndex = m_columnIndices[el];
            const double *KElement = elmStiffness[el].data();

            // add KElement to the global stiffness matrix
            for (int i=0; i<4; i++)
                for (int j=0; j<4; j++)
                    for(int k=0; k<3; k++)
                        for(int l=0; l<3; l++)
                            stiffnessMatrix.AddEntry(3 * rowIndex[i] + k, 3 * columnIndex[4 * i + j] + l, KElement[12 * (3 * i + k) + 3 * j + l]);
        }
   } 

    
    bool VegaFEMInterface::DoTimeStep(double t, StepTask &task)
    {
        double *q = reinterpret_cast<double *>(task.nodalPosition.data());
        double *qvel = reinterpret_cast<double *>(task.nodalVelocity.data());
        const double *internalForces = reinterpret_cast<const double *>(task.nodalForce.data());
        const double *externalForces = reinterpret_cast<const double *>(task.externalForce.data());
        const double *gravityForces = reinterpret_cast<const double *>(task.gravityForce.data());

        double *qresidual = m_qresidual.data();
        double *qdelta = m_qdelta.data();
        double *bufferConstrained = m_bufferConstrained.data();
        double *buffer = m_buffer.data();

        AssembleStiffnessMatrix(task.elmStiffness);
        if (task.dampingStiffnessCoef != 0)
            m_tangentStiffnessMatrix->ScalarMultiply(task.dampingStiffnessCoef, m_rayleighDampingMatrix.get());

        if (task.dampingMassCoef != 0)
            m_rayleighDampingMatrix->AddSubMatrix(task.dampingMassCoef, *m_massMatrix);

        
        *m_tangentStiffnessMatrix *= t;
        
        if (task.dampingStiffnessCoef != 0)
            *m_tangentStiffnessMatrix += *m_rayleighDampingMatrix;

        m_tangentStiffnessMatrix->MultiplyVector(qvel, qresidual);
        *m_tangentStiffnessMatrix *= t;
        m_tangentStiffnessMatrix->AddSubMatrix(1.0, *m_massMatrix);
                      
        int r = m_dof;
        for(int i = 0; i < r; ++i)
        {
            qresidual[i] += internalForces[i] - externalForces[i] - gravityForces[i];
            qresidual[i] *= -t;
            qdelta[i] = qresidual[i];
        }
        
        RemoveRows(r, bufferConstrained, qdelta, (int) m_constrainedDOFs.size(), m_constrainedDOFs.data());
        m_systemMatrix->AssignSuperMatrix(m_tangentStiffnessMatrix.get());
        memset(buffer, 0, sizeof(double) * r);
         
#define UseCG 0
#if UseCG
        int info = m_CGSolver->SolveLinearSystemWithJacobiPreconditioner(buffer, bufferConstrained, 1e-6, 10000);
        //int info = m_CGSolver->SolveLinearSystemWithoutPreconditioner(buffer, bufferConstrained, 1e-6, 10000);
        if (info < 0)
            printf("Error: %s sparse solver returned non-zero exit status %d.\n", "PCG", (int)info);
#else
        //m_PardisoSolverSolver->SolveLinearSystem(buffer, bufferConstrained);
                        
        m_systemMatrix->GenerateCompressedRowMajorFormat(
            m_pardParam.a, NULL, NULL, 0, 1);

        //std::fill(m_bufferConstrained.begin(), m_bufferConstrained.end(), 0.0);
        //m_bufferConstrained[0] = 1e-10;
        m_pardParam.phase = 23;
        pardiso(
            m_pardParam.pt, 
            &m_pardParam.maxfct,
            &m_pardParam.mnum,
            &m_pardParam.mtype,
            &m_pardParam.phase,
            &m_pardParam.n, 
            m_pardParam.a, 
            m_pardParam.ia, 
            m_pardParam.ja, 
            NULL,//m_pardParam.perm,
            &m_pardParam.nrhs, 
            m_pardParam.iparm, 
            &m_pardParam.msglvl, 
            bufferConstrained, 
            buffer, 
            &m_pardParam.error);

        if (m_pardParam.error != 0)
		{
            printf("Error: Pardiso solve returned non-zero exit code %d.\n", m_pardParam.error);
			memset(buffer, 0, sizeof(double) * r);
			return false;
		}

#endif

        //double result[10000];
        //memset(result, 0, sizeof(double) * 1000);
        //m_systemMatrix->MultiplyVector(buffer, result);
        //double total = 0;
        //for (int i = 0; i < r - (int) m_constrainedDOFs.size(); ++i)
        //{
        //    result[i] -= bufferConstrained[i];
        //    total += abs(result[i]);
        //}
        //printf("res: %0.10g\n", total);

        InsertRows(r, buffer, qdelta, (int) m_constrainedDOFs.size(), m_constrainedDOFs.data());
        
        for(int i = 0; i < r; ++i)
        {
            qvel[i] += qdelta[i];
            q[i] += t * qvel[i];
        }

		return true;
    }

	void VegaFEMInterface::ComputeNonLinearMode(int numDeriv, std::vector<double>& _freq, std::vector<double>& _modes, 
		const std::vector<double> &lumpedMass, double* sketchData, int numDataVectors)
	{
		int oneIndexed = 0;
		int n3 = 3 * m_tetTopology->GetNumVertices();

		// create mass matrix
		//SparseMatrix* massMatrix(*(m_massMatrix.get()));
		SparseMatrix* massMatrix = new SparseMatrix(*(m_massMatrix.get()));
		//massMatrix->RemoveRowsColumns(m_constrainedDOFs.size(), m_constrainedDOFs.data(), oneIndexed);

		// mass-normalize modal derivatives
		//double * modalDerivatives/* = precomputationState.modalDerivativesMatrix->GetMatrix()*/;
		//double * normalizedModalDerivatives = (double*) malloc (sizeof(double) * n3 * numDeriv);
		//memcpy(normalizedModalDerivatives, modalDerivatives, sizeof(double) * n3 * numDeriv);
	 //   for(int i=0; i < numDeriv; i++)
		//	massMatrix->NormalizeVector(&(normalizedModalDerivatives[n3 * i]));


		// data from external simulation
		double * dataMatrix;
		//numDataVectors = precomputationState.sketchDataMatrix->Getr();
		dataMatrix = (double*) malloc (sizeof(double) * n3 * numDataVectors);
		memcpy(dataMatrix, sketchData, sizeof(double) * n3 * numDataVectors);


		// do lumped-mass-PCA on dataMatrix ( n3 x numDataVectors )
		double * ones = (double*) malloc (sizeof(double) * n3);
		for(int i=0; i<n3; i++)
			ones[i] = 1.0;

		double * LTDiagonal = (double*) malloc (sizeof(double) * n3);
		massMatrix->MultiplyVector(ones, LTDiagonal);
		free(ones);

		// sqrt
		for(int i=0; i<n3; i++)
			LTDiagonal[i] = sqrt(LTDiagonal[i]);

		//// number of retained dimensions can't be more than num linear modes + num derivatives
		//if (uiState.numComputedNonLinearModes > numDataVectors)
		//	uiState.numComputedNonLinearModes = numDataVectors;

		// premultiply by LT
		for(int i=0; i<n3; i++)
			for(int j=0; j < numDataVectors; j++)
				dataMatrix[ELT(n3, i, j)] *= LTDiagonal[i];

		ThresholdingSpecification thresholdingSpecification;
		thresholdingSpecification.tresholdingType = ThresholdingSpecification::numberOfModesBased;
		thresholdingSpecification.rDesired = numDeriv;

		int outputr;
		int matrixPCACode = 0;
		if ( ((matrixPCACode = MatrixPCA(
			&thresholdingSpecification, n3, numDataVectors, dataMatrix, &outputr)) != 0) 
			|| (outputr != numDeriv))
		{
			printf("Error performing SVD. Code: %d\n", matrixPCACode);
			//*code = matrixPCACode;
			free(dataMatrix);
			free(LTDiagonal);
			delete massMatrix;
			return;
		}

		// solve L^T U = V
		for(int i=0; i<n3; i++)
			for(int j=0; j < numDeriv; j++)
				dataMatrix[ELT(n3, i, j)] /= LTDiagonal[i];

		// export data
		memcpy(&_modes[0], dataMatrix, sizeof(double)*n3*numDeriv);
	/*	*modes_ = (double*) realloc (dataMatrix, 
			sizeof(double) * n3 * uiState.numComputedNonLinearModes);
*/
		free(LTDiagonal);
		free(dataMatrix);
		delete massMatrix;
		return;
	}


		void VegaFEMInterface::ComputeLinearMode(int numDesiredMode, const std::vector<std::array<double, 12*12>> &elmUndeformedStiffness, std::vector<double>& _freq, 
			std::vector<double>& _modes, const std::vector<double> &lumpedMass)
	{
		int oneIndexed = 0;

		// create mass matrix
		//SparseMatrix* massMatrix(*(m_massMatrix.get()));
		SparseMatrix* massMatrix = new SparseMatrix(*(m_massMatrix.get()));
		massMatrix->RemoveRowsColumns(m_constrainedDOFs.size(), m_constrainedDOFs.data(), oneIndexed);

		// create stiffness matrix
		//SparseMatrix* stiffnessMatrix = new SparseMatrix(*(m_tangentStiffnessMatrix.get()));
		AssembleUndeformedStiffnessMatrix(elmUndeformedStiffness);
		SparseMatrix* stiffnessMatrix = new SparseMatrix(*(m_undeformedStiffnessMatrix.get()));
		stiffnessMatrix->RemoveRowsColumns(m_constrainedDOFs.size(), m_constrainedDOFs.data(), oneIndexed);

		// call ARPACK

		double * frequenciesTemp = (double*) malloc (sizeof(double) * numDesiredMode);
		int numRetainedDOFs = stiffnessMatrix->Getn();
		double * modesTemp = (double*) malloc 
			(sizeof(double) * numDesiredMode * numRetainedDOFs);

		//printf("Computing linear modes using ARPACK: ...\n");
		//PerformanceCounter ARPACKCounter;
		double sigma = -1.0;

		//int numLinearSolverThreads = wxThread::GetCPUCount();
		//if (numLinearSolverThreads > 3)
		//	numLinearSolverThreads = 3; // diminished returns in solver beyond 3 threads

		ARPACKSolver generalizedEigenvalueProblem;
		int nconv = generalizedEigenvalueProblem.SolveGenEigShInv
			(stiffnessMatrix, massMatrix, 
			numDesiredMode, frequenciesTemp, 
			modesTemp, sigma, /*numLinearSolverThreads*/1);

		//////////////////////////////////////////
		//QString score_filePath = QDir::toNativeSeparators("KNoGravity.txt");
		//tofstream fout_warmstart;
		//fout_warmstart.open(score_filePath.utf16());
		//fout_warmstart.precision(18);

		//double* densMatrix = new double[numRetainedDOFs*numRetainedDOFs];
		//stiffnessMatrix->MakeDenseMatrix(densMatrix);
		//for(int i=0; i<numRetainedDOFs; i++)
		//{
		//	for(int j=0; j<numRetainedDOFs; j++)
		//	{
		//		fout_warmstart << densMatrix[i*numRetainedDOFs+j] << " " ;
		//	}
		//	fout_warmstart << std::endl;
		//}

		//fout_warmstart.close();
		//delete[] densMatrix;
		//////////////////////////////////////////////////
		//QString score_filePath1 = QDir::toNativeSeparators("massMatrixNoGravity.txt");
		//tofstream fout_mass;
		//fout_mass.open(score_filePath1.utf16());
		//fout_mass.precision(18);

		//double* masMatrix = new double[numRetainedDOFs*numRetainedDOFs];
		//massMatrix->MakeDenseMatrix(masMatrix);
		//for(int i=0; i<numRetainedDOFs; i++)
		//{
		//	for(int j=0; j<numRetainedDOFs; j++)
		//	{
		//		fout_mass << masMatrix[i*numRetainedDOFs+j] << " " ;
		//	}
		//	fout_mass << std::endl;
		//}

		//delete[] masMatrix;
		//fout_mass.close();
		/////////////////////////////////////////////////////
		//QString score_filePath2 = QDir::toNativeSeparators("constraintNoGravity.txt");
		//tofstream fout_constraint;
		//fout_constraint.open(score_filePath2.utf16());
		//fout_constraint.precision(18);

		//for(int i=0; i<m_constrainedDOFs.size(); i++)
		//		fout_constraint << m_constrainedDOFs[i] << " " ;

		//fout_constraint << std::endl;

		//fout_constraint.close();
		////////////////////////////////////////////////////////

		//ARPACKCounter.StopCounter();
		//double ARPACKTime = ARPACKCounter.GetElapsedTime();
		//printf("ARPACK time: %G s.\n", ARPACKTime); fflush(NULL);

		// can not find eigen mode
		if (nconv < numDesiredMode)
		{
			free(modesTemp);
			free(frequenciesTemp);
			//*r = -3;
			delete(massMatrix);
			delete(stiffnessMatrix);
			return;
			std::cout << "can not find eigen vector" << std::endl;
		}

		int n3 = 3 * m_tetTopology->GetNumVertices();

		for(int i=0; i<numDesiredMode; i++)
		{
			// insert zero rows into the computed modes
			int oneIndexed = 0;
			InsertRows(n3, &modesTemp[numRetainedDOFs*i], &(_modes[n3*i]), 
				m_constrainedDOFs.size(), m_constrainedDOFs.data(), oneIndexed);
		}

		for(int i=0; i<numDesiredMode; i++)
		{
			if (frequenciesTemp[i] <= 0)
				_freq[i] = 0.0;
			else
				_freq[i] = sqrt((frequenciesTemp)[i]) / (2 * M_PI);
		}

		//for(int i=0; i<numDesiredMode; i++)
		//{
		//	int index = i*n3;
		//	for(int j=0; j<m_tetTopology->GetNumVertices(); j++)
		//	{
		//		double mass = lumpedMass[j];
		//		_modes[i][j] = _modes[i][j]*mass;
		//		/*_modes[i][j] = _modes[index+j*3+1]*mass;
		//		_modes[index+j*3+2] = _modes[index+j*3+2]*mass;*/
		//	}
		//}

		free(modesTemp);
		free(frequenciesTemp);

		delete(massMatrix);
		delete(stiffnessMatrix);

		//*r = numDesiredModes;

		return;


	}

}