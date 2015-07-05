#include "stdafx.h"
#include "CorotatedLinearModel.h"

#include "PolarDecomposition.h"


namespace Libin
{

    // for debug
    static void DumpMatrix(const char *filename, double *M, int row, int col)
    {
        FILE *file = NULL;
        fopen_s(&file, filename, "w");
        if (!file)
        {
            printf("Can't open %s\n", filename);
            return;
        }

        for (int i = 0; i < row; ++i)
        {
            for (int j = 0; j < col; ++j)
                fprintf(file, "%0.20g\t", *(M++));
            fprintf(file, "\n");
        }

        fclose(file);
    }

    static void PrintMatrix(double *M, int row, int col, const char *format = "%0.20g\t")
    {
        printf("-------------------------------------------------------------------------\n");
        for (int i = 0; i < row; ++i)
        {
            for (int j = 0; j < col; ++j)
                printf(format, *(M++));
            printf("\n");
        }
        printf("-------------------------------------------------------------------------\n");
    }

    CorotatedLinearModel::CorotatedLinearModel(std::shared_ptr<TetMeshTopology> topology)
        : m_topology(topology)
    {
    }


    CorotatedLinearModel::~CorotatedLinearModel(void)
    {
    }


    void CorotatedLinearModel::InitializeFirstPiolaKirchhoffMethod(TetMesh *pTetMesh)
    {    
        auto &vertices = pTetMesh->GetVertices();
        auto &elements = m_topology->GetElements();
        auto &material = pTetMesh->GetMaterial();
        m_elmVolume = pTetMesh->GetElementVolume();

        size_t numElm = elements.size();
        size_t numVert = vertices.size();
        m_elmDs.resize(numElm);
        m_elmDm.resize(numElm);
        m_elmBm.resize(numElm);  // Bm=Dm^-1
        m_elmWBmT.resize(numElm);   // w * Bm

        m_elmF.resize(numElm);
        m_elmR.resize(numElm);
        m_elmS.resize(numElm);
        m_flagFRSCalculated.resize(numElm, false);

        std::array<Vector3d, 4> fz = { Vector3d::ZERO, Vector3d::ZERO, Vector3d::ZERO, Vector3d::ZERO };
        m_elmElasticForces.resize(numElm, fz);
        m_flagElmElasticForceCalculated.resize(numElm, false);
    
        m_elmStiffMu.resize(numElm);
        m_elmStiffLambda.resize(numElm);

        for (int el = 0; el < numElm; ++el)
        {
            //m_elmVolume[el] = elmVol[el];
            m_elmStiffMu[el] = material[el].Mu();
            m_elmStiffLambda[el] = material[el].Lambda();

            auto &vertIdx = elements[el];
        
            const Vector3d &v3 = vertices[vertIdx[3]];
            for (int i = 0; i < 3; ++i)
                m_elmDm[el].SetColumn(i, vertices[vertIdx[i]] - v3);

            m_elmBm[el] = m_elmDm[el].Inverse();
            m_elmWBmT[el] = m_elmVolume[el] * m_elmBm[el].Transpose();
        }
        
        ResetAllFlags();
    }

#pragma region BEMat
    // the shape matrix B is a sparse 6x12 matrix
    // we don't compute and store this matrix explicitly, we only compute multipication with B
    inline static double *BTransposeTimesMatrix(const Matrix4d &VBm, const double *M, size_t mcol, double *BtM)
    {
        const double *MInv = VBm;
        //double B[72] = 
        //{ 
        //    MInv[0], 0, 0,  MInv[4], 0, 0,  MInv[8], 0, 0,   MInv[12], 0, 0,
        //    0, MInv[1], 0,  0, MInv[5], 0,  0, MInv[9], 0,   0, MInv[13], 0,
        //    0, 0, MInv[2],  0, 0, MInv[6],  0, 0, MInv[10],  0, 0, MInv[14],

        //    MInv[1 ], MInv[0 ], 0,   MInv[5 ], MInv[4 ], 0,  MInv[9 ], MInv[8 ], 0,   MInv[13], MInv[12], 0,  
        //    0, MInv[2 ], MInv[1 ],   0, MInv[6 ], MInv[5 ],  0, MInv[10], MInv[9 ],   0, MInv[14], MInv[13],  
        //    MInv[2 ], 0, MInv[0 ],   MInv[6 ], 0, MInv[4 ],  MInv[10], 0, MInv[8 ],   MInv[14], 0, MInv[12]
        //};
        memset(BtM, 0, sizeof(double) * 12 * mcol);
        double *BtMrows = BtM;
        const double *Mrows[] = { M, M + mcol, M + mcol * 2, M + mcol * 3, M + mcol * 4, M + mcol * 5 };
        for (int i = 0; i < 4; ++i, MInv += 4)
        {
            for (size_t j = 0; j < mcol; ++j)
                *(BtMrows++) = MInv[0] * Mrows[0][j]    + MInv[1] * Mrows[3][j] + MInv[2] * Mrows[5][j];
            for (size_t j = 0; j < mcol; ++j)
                *(BtMrows++) = MInv[1] * Mrows[1][j]    + MInv[0] * Mrows[3][j] + MInv[2] * Mrows[4][j];
            for (size_t j = 0; j < mcol; ++j)
                *(BtMrows++) = MInv[2] * Mrows[2][j]    + MInv[1] * Mrows[4][j] + MInv[0] * Mrows[5][j];
        }

        return BtM;
    }
    inline static double *BTimesMatrix(const Matrix4d &VBm, const double *M, size_t mcol, double *BM)
    {
        const double *MInv = VBm;
        //double B[72] = 
        //{ 
        //    MInv[0], 0, 0,  MInv[4], 0, 0,  MInv[8], 0, 0,   MInv[12], 0, 0,
        //    0, MInv[1], 0,  0, MInv[5], 0,  0, MInv[9], 0,   0, MInv[13], 0,
        //    0, 0, MInv[2],  0, 0, MInv[6],  0, 0, MInv[10],  0, 0, MInv[14],

        double Blow[36] = {
            MInv[1 ], MInv[0 ], 0,   MInv[5 ], MInv[4 ], 0,  MInv[9 ], MInv[8 ], 0,   MInv[13], MInv[12], 0,  
            0, MInv[2 ], MInv[1 ],   0, MInv[6 ], MInv[5 ],  0, MInv[10], MInv[9 ],   0, MInv[14], MInv[13],  
            MInv[2 ], 0, MInv[0 ],   MInv[6 ], 0, MInv[4 ],  MInv[10], 0, MInv[8 ],   MInv[14], 0, MInv[12]
        };
        memset(BM, 0, sizeof(double) * 6 * mcol);
        double *BMrows = BM;
        const double *Mrows[12];
        const double *temp = M;
        for (int i = 0; i < 12; ++i, temp += mcol)
            Mrows[i] = temp;

        for (int j = 0; j < mcol; ++j, BMrows += mcol)
        {
            BMrows[0] = MInv[0] * Mrows[0][j] + MInv[4] * Mrows[3][j] + MInv[8 ] * Mrows[6][j] + MInv[12] * Mrows[9 ][j];
            BMrows[1] = MInv[1] * Mrows[1][j] + MInv[5] * Mrows[4][j] + MInv[9 ] * Mrows[7][j] + MInv[13] * Mrows[10][j];
            BMrows[2] = MInv[2] * Mrows[2][j] + MInv[6] * Mrows[5][j] + MInv[10] * Mrows[8][j] + MInv[14] * Mrows[11][j];

            for (int i = 0; i < 3; ++i)
            {
                BMrows[3 + i] = 0;
                for (int k = 0; k < 12; ++k)
                    BMrows[3 + i] += Blow[i * 12 + k] * Mrows[k][j];
            }

            for (int i = 0; i < 12; ++i)
                ++Mrows[i];
        }

        return BM;
    }

    inline static double *MatrixTimesB(const Matrix4d &VBm, const double *M, size_t mrow, double *MB)
    {
        const double *MInv0 = VBm;
        //double B[72] = 
        //{ 
        //    MInv[0], 0, 0,  MInv[4], 0, 0,  MInv[8], 0, 0,   MInv[12], 0, 0,
        //    0, MInv[1], 0,  0, MInv[5], 0,  0, MInv[9], 0,   0, MInv[13], 0,
        //    0, 0, MInv[2],  0, 0, MInv[6],  0, 0, MInv[10],  0, 0, MInv[14],

        //    MInv[1 ], MInv[0 ], 0,   MInv[5 ], MInv[4 ], 0,  MInv[9 ], MInv[8 ], 0,   MInv[13], MInv[12], 0,  
        //    0, MInv[2 ], MInv[1 ],   0, MInv[6 ], MInv[5 ],  0, MInv[10], MInv[9 ],   0, MInv[14], MInv[13],  
        //    MInv[2 ], 0, MInv[0 ],   MInv[6 ], 0, MInv[4 ],  MInv[10], 0, MInv[8 ],   MInv[14], 0, MInv[12]
        //};
        memset(MB, 0, sizeof(double) * 12 * mrow);
        double *MBrow = MB;
        const double *Mcols[] = { M, M + 1, M + 2, M + 3, M + 4, M + 5 };

        for (size_t j = 0; j < mrow; ++j)
        {
            const double *MInv = MInv0;
            for (int i = 0; i < 4; ++i, MInv += 4)
            {
                *(MBrow++) = MInv[0] * *Mcols[0]    + MInv[1] * *Mcols[3] + MInv[2] * *Mcols[5];
                *(MBrow++) = MInv[1] * *Mcols[1]    + MInv[0] * *Mcols[3] + MInv[2] * *Mcols[4];
                *(MBrow++) = MInv[2] * *Mcols[2]    + MInv[1] * *Mcols[4] + MInv[0] * *Mcols[5];
            }

            for (int i = 0; i < 6; ++i)
                Mcols[i] += 6;
        }

        return MB;
    }

    inline static double *ComputeBtE(const Matrix4d &VBm, double lambda, double mu, double *BtE)
    {
        const double *MInv = VBm;
        //double B[72] = 
        //{ 
        //    MInv[0], 0, 0,  MInv[4], 0, 0,  MInv[8], 0, 0,   MInv[12], 0, 0,
        //    0, MInv[1], 0,  0, MInv[5], 0,  0, MInv[9], 0,   0, MInv[13], 0,
        //    0, 0, MInv[2],  0, 0, MInv[6],  0, 0, MInv[10],  0, 0, MInv[14],

        //    MInv[1 ], MInv[0 ], 0,   MInv[5 ], MInv[4 ], 0,  MInv[9 ], MInv[8 ], 0,   MInv[13], MInv[12], 0,  
        //    0, MInv[2 ], MInv[1 ],   0, MInv[6 ], MInv[5 ],  0, MInv[10], MInv[9 ],   0, MInv[14], MInv[13],  
        //    MInv[2 ], 0, MInv[0 ],   MInv[6 ], 0, MInv[4 ],  MInv[10], 0, MInv[8 ],   MInv[14], 0, MInv[12]
        //};
        // Bt E is 12x6
        memset(BtE, 0, sizeof(double) * 12 * 6);
        double *BtErow = BtE;
        double lmbdmu = lambda + 2 * mu;
        for (int i = 0; i < 4; ++i, MInv += 4)
        {
            *(BtErow++) = lmbdmu * MInv[0]; *(BtErow++) = lambda * MInv[0]; *(BtErow++) = lambda * MInv[0]; 
            *(BtErow++) = mu * MInv[1];     *(BtErow++) = 0.0;              *(BtErow++) = mu * MInv[2];

            *(BtErow++) = lambda * MInv[1]; *(BtErow++) = lmbdmu * MInv[1]; *(BtErow++) = lambda * MInv[1];
            *(BtErow++) = mu * MInv[0];     *(BtErow++) = mu * MInv[2];     *(BtErow++) = 0.0;
        
            *(BtErow++) = lambda * MInv[2]; *(BtErow++) = lambda * MInv[2]; *(BtErow++) = lmbdmu * MInv[2];
            *(BtErow++) = 0.0;              *(BtErow++) = mu * MInv[1];     *(BtErow++) = mu * MInv[0];
        }

        return BtE;
    }

    inline static double *ComputeEB(const Matrix4d &VBm, double lambda, double mu, double *EB)
    {
        const double *MInv = VBm;
        //double B[72] = 
        //{ 
        //    MInv[0], 0, 0,  MInv[4], 0, 0,  MInv[8], 0, 0,   MInv[12], 0, 0,
        //    0, MInv[1], 0,  0, MInv[5], 0,  0, MInv[9], 0,   0, MInv[13], 0,
        //    0, 0, MInv[2],  0, 0, MInv[6],  0, 0, MInv[10],  0, 0, MInv[14],

        //    MInv[1 ], MInv[0 ], 0,   MInv[5 ], MInv[4 ], 0,  MInv[9 ], MInv[8 ], 0,   MInv[13], MInv[12], 0,  
        //    0, MInv[2 ], MInv[1 ],   0, MInv[6 ], MInv[5 ],  0, MInv[10], MInv[9 ],   0, MInv[14], MInv[13],  
        //    MInv[2 ], 0, MInv[0 ],   MInv[6 ], 0, MInv[4 ],  MInv[10], 0, MInv[8 ],   MInv[14], 0, MInv[12]
        //};
        memset(EB, 0, sizeof(double) * 6 * 12);
        double lmbdmu = lambda + 2 * mu;
        double *EBrows[] = { EB, EB + 12, EB + 24, EB + 36, EB + 48, EB + 60 };

        for (int i = 0; i < 4; ++i, MInv += 4)
        {
            *(EBrows[0]++) = lmbdmu * MInv[0];  *(EBrows[1]++) = lambda * MInv[0];  *(EBrows[2]++) = lambda * MInv[0];
            *(EBrows[3]++) = mu * MInv[1];      *(EBrows[4]++) = 0.0;               *(EBrows[5]++) = mu * MInv[2];
        
            *(EBrows[0]++) = lambda * MInv[1];  *(EBrows[1]++) = lmbdmu * MInv[1];  *(EBrows[2]++) = lambda * MInv[1];
            *(EBrows[3]++) = mu * MInv[0];      *(EBrows[4]++) = mu * MInv[2];      *(EBrows[5]++) = 0.0;
        
            *(EBrows[0]++) = lambda * MInv[2];  *(EBrows[1]++) = lambda * MInv[2];  *(EBrows[2]++) = lmbdmu * MInv[2];
            *(EBrows[3]++) = 0.0;               *(EBrows[4]++) = mu * MInv[1];      *(EBrows[5]++) = mu * MInv[0];
        }
    
        return EB;
    }

    inline static double *ComputeBtGB(const Matrix4d &VBm, double *BtGB)
    {
        const double *MInv = VBm;
        //double B[72] = 
        //{ 
        //    MInv[0 ],        0,        0,  MInv[4], 0, 0,  MInv[8], 0, 0,   MInv[12], 0, 0,
        //           0, MInv[1 ],        0,  0, MInv[5], 0,  0, MInv[9], 0,   0, MInv[13], 0,
        //           0,        0, MInv[2 ],  0, 0, MInv[6],  0, 0, MInv[10],  0, 0, MInv[14],

        //    MInv[1 ], MInv[0 ],        0,   MInv[5 ], MInv[4 ], 0,  MInv[9 ], MInv[8 ], 0,   MInv[13], MInv[12], 0,  
        //           0  MInv[2 ], MInv[1 ],   0, MInv[6 ], MInv[5 ],  0, MInv[10], MInv[9 ],   0, MInv[14], MInv[13],  
        //    MInv[2 ],        0, MInv[0 ],   MInv[6 ], 0, MInv[4 ],  MInv[10], 0, MInv[8 ],   MInv[14], 0, MInv[12]
        //};
        // G = diag(1, 1, 1, 0.5, 0.5, 0.5)
        memset(BtGB, 0, sizeof(double) * 12 * 12);
        double *entry = BtGB;

        const double *MInvi = MInv;
        for (int i = 0; i < 4; ++i, MInvi += 4)
        {
            const double *MInvj;
            MInvj = MInv;
            for (int j = 0; j < 4; ++j, MInvj += 4)
            {
                *(entry++) = MInvi[0] * MInvj[0] + (MInvi[1 ]* MInvj[1 ] + MInvi[2 ] * MInvj[2 ]) * 0.5;
                *(entry++) = (MInvi[1 ]* MInvj[0 ]) * 0.5;
                *(entry++) = (MInvi[2 ]* MInvj[0 ]) * 0.5;
            }
            MInvj = MInv;
            for (int j = 0; j < 4; ++j, MInvj += 4)
            {
                *(entry++) = (MInvi[0 ]* MInvj[1 ]) * 0.5;
                *(entry++) = MInvi[1] * MInvj[1] + (MInvi[0 ]* MInvj[0 ] + MInvi[2 ] * MInvj[2 ]) * 0.5;
                *(entry++) = (MInvi[2 ]* MInvj[1 ]) * 0.5;
            }
            MInvj = MInv;
            for (int j = 0; j < 4; ++j, MInvj += 4)
            {
                *(entry++) = (MInvi[0 ]* MInvj[2 ]) * 0.5;
                *(entry++) = (MInvi[1 ]* MInvj[2 ]) * 0.5;
                *(entry++) = MInvi[2] * MInvj[2] + (MInvi[1 ]* MInvj[1 ] + MInvi[0 ] * MInvj[0 ]) * 0.5;
            }
        }

        return BtGB;
    }

    inline static double *ComputeBtH(const Matrix4d &VBm, double *BtH)
    {
        // H[72] = 
        //{
        //    -1,  1,  1,
        //     1, -1,  1,
        //     1,  1, -1,
        //                -1,
        //                    -1,
        //                        -1
        //}
        const double *MInv = VBm;
        //double B[72] = 
        //{ 
        //    MInv[0], 0, 0,  MInv[4], 0, 0,  MInv[8], 0, 0,   MInv[12], 0, 0,
        //    0, MInv[1], 0,  0, MInv[5], 0,  0, MInv[9], 0,   0, MInv[13], 0,
        //    0, 0, MInv[2],  0, 0, MInv[6],  0, 0, MInv[10],  0, 0, MInv[14],

        //    MInv[1 ], MInv[0 ], 0,   MInv[5 ], MInv[4 ], 0,  MInv[9 ], MInv[8 ], 0,   MInv[13], MInv[12], 0,  
        //    0, MInv[2 ], MInv[1 ],   0, MInv[6 ], MInv[5 ],  0, MInv[10], MInv[9 ],   0, MInv[14], MInv[13],  
        //    MInv[2 ], 0, MInv[0 ],   MInv[6 ], 0, MInv[4 ],  MInv[10], 0, MInv[8 ],   MInv[14], 0, MInv[12]
        //};
        // Bt E is 12x6
        memset(BtH, 0, sizeof(double) * 12 * 6);
        double *BtHrow = BtH;
        for (int i = 0; i < 4; ++i, MInv += 4)
        {
            *(BtHrow++) = -MInv[0];     *(BtHrow++) =  MInv[0];     *(BtHrow++) =  MInv[0]; 
            *(BtHrow++) = -MInv[1];     *(BtHrow++) =  0.0;         *(BtHrow++) = -MInv[2];

            *(BtHrow++) =  MInv[1];     *(BtHrow++) = -MInv[1];     *(BtHrow++) =  MInv[1];
            *(BtHrow++) = -MInv[0];     *(BtHrow++) = -MInv[2];     *(BtHrow++) =  0.0;
        
            *(BtHrow++) =  MInv[2];     *(BtHrow++) =  MInv[2];     *(BtHrow++) = -MInv[2];
            *(BtHrow++) =  0.0;         *(BtHrow++) = -MInv[1];     *(BtHrow++) = -MInv[0];
        }

        return BtH;
    }
#pragma endregion 

    inline static double RoundByText(double d)
    {
        char buf[100];
        sprintf_s(buf, "%0.10g", d);
        d = 0;
        std::stringstream(buf) >> d;
        return d;
    }

    static void TestBEFunctions(void)
    {
        Matrix4d M(
            RoundByText(Mathd::SymmetricRandom()), RoundByText(Mathd::SymmetricRandom()), RoundByText(Mathd::SymmetricRandom()), RoundByText(Mathd::SymmetricRandom()),
            RoundByText(Mathd::SymmetricRandom()), RoundByText(Mathd::SymmetricRandom()), RoundByText(Mathd::SymmetricRandom()), RoundByText(Mathd::SymmetricRandom()),
            RoundByText(Mathd::SymmetricRandom()), RoundByText(Mathd::SymmetricRandom()), RoundByText(Mathd::SymmetricRandom()), RoundByText(Mathd::SymmetricRandom()),
            1.0, 1.0, 1.0, 1.0
            );

        printf("M\n");
        PrintMatrix(M, 4, 4, "%20.15g ");

        auto Bm = M.Inverse();
        double *MInv = Bm;

        TetMeshMaterials::Material mt = { 1e7, 0.45, 1000 };
        double lambda = mt.Lambda();
        double mu = mt.Mu();

        double B[72] = 
        { 
            MInv[0], 0, 0,  MInv[4], 0, 0,  MInv[8], 0, 0,   MInv[12], 0, 0,
            0, MInv[1], 0,  0, MInv[5], 0,  0, MInv[9], 0,   0, MInv[13], 0,
            0, 0, MInv[2],  0, 0, MInv[6],  0, 0, MInv[10],  0, 0, MInv[14],

            MInv[1 ], MInv[0 ], 0,   MInv[5 ], MInv[4 ], 0,  MInv[9 ], MInv[8 ], 0,   MInv[13], MInv[12], 0,  
            0, MInv[2 ], MInv[1 ],   0, MInv[6 ], MInv[5 ],  0, MInv[10], MInv[9 ],   0, MInv[14], MInv[13],  
            MInv[2 ], 0, MInv[0 ],   MInv[6 ], 0, MInv[4 ],  MInv[10], 0, MInv[8 ],   MInv[14], 0, MInv[12]
        };
    
        printf("B\n");
        PrintMatrix(B, 6, 12, "%20.15g ");
    
        // compute stiffness matrix
        double E[36] = {
            lambda + 2 * mu, lambda, lambda, 0, 0, 0,
            lambda, lambda + 2 * mu, lambda, 0, 0, 0,
            lambda, lambda, lambda + 2 * mu, 0, 0, 0,
            0,      0,      0,               mu, 0, 0,
            0,      0,      0,               0, mu, 0,
            0,      0,      0,               0, 0, mu 
        };
        printf("E\n");
        PrintMatrix(E, 6, 6, "%20.15g ");
    

        double BtE[72], EB[72], BtE2[72], EB2[72];
        ComputeBtE(Bm, lambda, mu, BtE);
        ComputeEB(Bm, lambda, mu, EB);
    
        printf("BtE\n");
        PrintMatrix(BtE, 12, 6, "%20.15g ");
        printf("EB\n");
        PrintMatrix(EB, 6, 12, "%20.15g ");

        BTransposeTimesMatrix(Bm, E, 6, BtE2);
        MatrixTimesB(Bm, E, 6, EB2);
    
        printf("BtE2\n");
        PrintMatrix(BtE2, 12, 6, "%20.15g ");
        printf("EB2\n");
        PrintMatrix(EB2, 6, 12, "%20.15g ");
    
        for (int i = 0; i < 72; ++i)
            BtE2[i] -= BtE[i];
        for (int i = 0; i < 72; ++i)
            EB2[i] -= EB[i];

        printf("BtE Diff\n");
        PrintMatrix(BtE2, 12, 6, "%20.15g ");
        printf("EB Diff\n");
        PrintMatrix(EB2, 6, 12, "%20.15g ");

        double X[12];
        for (int i = 0; i < 12; ++i)
            X[i] = Mathd::SymmetricRandom() * 0.1;
        double BX[6];
        BTimesMatrix(Bm, X, 1, BX);
    
        printf("X\n");
        PrintMatrix(X, 12, 1, "%20.15g ");
        printf("BX\n");
        PrintMatrix(BX, 6, 1, "%20.15g ");
    }

    void CorotatedLinearModel::InitializeUndeformedStiffnessMatrix(TetMesh *pTetMesh)
    {
        auto &vertices = pTetMesh->GetVertices();
        auto &elements = m_topology->GetElements();
        auto &material = pTetMesh->GetMaterial();
        m_elmVolume = pTetMesh->GetElementVolume();

        size_t numElm = elements.size();
        size_t numVert = vertices.size();
    
        m_elmVs.resize(numElm);
        m_elmVm.resize(numElm);
        m_elmVBm.resize(numElm);    

        //m_elmVolume.resize(numElm);
        //m_elmStiffMu.resize(numElm);
        //m_elmStiffLambda.resize(numElm);   
    
        m_elmF.resize(numElm);
        m_elmR.resize(numElm);
        m_elmS.resize(numElm);
        m_flagFRSCalculated.resize(numElm, false);

        std::array<Vector3d, 4> fz = { Vector3d::ZERO, Vector3d::ZERO, Vector3d::ZERO, Vector3d::ZERO };
        m_elmElasticForces.resize(numElm, fz);
        m_flagElmElasticForceCalculated.resize(numElm, false);

        m_elmStiffness.resize(numElm);
        m_elmUndeformedStiffness.resize(numElm);
        m_elmMaterialStiffness.resize(numElm);
        m_elmVolBtE.resize(numElm);
    
        m_flagElmStifnessCalculated.resize(numElm, false);
        m_flagElmMaterialStiffnessCalculated.resize(numElm, false);

        m_elmBtGB.resize(numElm);
        m_elmBtHB.resize(numElm);
        m_elmDfDalpha.resize(numElm);
        m_elmDfDlambda.resize(numElm);
        m_flagDfDlambdaDalphaCalculated.resize(numElm, false);

        double BtH[72];

        for (int el = 0; el < numElm; ++el)
        {        
            auto &vertIdx = elements[el];
            // set element Vm
            auto &Vm = m_elmVm[el];
            for (int vi = 0; vi < 4; ++vi)
                Vm.SetColumn(vi, Vector4d(vertices[vertIdx[vi]], 1.0));
            m_elmVBm[el] = Vm.Inverse();

            double vol = m_elmVolume[el];
            double lambda = material[el].Lambda() * vol;
            double mu = material[el].Mu() * vol;

            //// compute stiffness matrix
            //double E[36] = {
            //    lambda + 2 * mu, lambda, lambda, 0, 0, 0,
            //    lambda, lambda + 2 * mu, lambda, 0, 0, 0,
            //    lambda, lambda, lambda + 2 * mu, 0, 0, 0,
            //    0,      0,      0,               mu, 0, 0,
            //    0,      0,      0,               0, mu, 0,
            //    0,      0,      0,               0, 0, mu 
            //};
            ComputeBtE(m_elmVBm[el], lambda, mu, m_elmVolBtE[el].data());
            //MatrixTimesB(m_elmVBm[el], m_elmVolBtE[el].data(), 12, m_elmUndeformedStiffness[el].data());

            ComputeBtGB(m_elmVBm[el], m_elmBtGB[el].data());
            ComputeBtH(m_elmVBm[el], BtH);
            MatrixTimesB(m_elmVBm[el], BtH, 12, m_elmBtHB[el].data());

            double llambda = material[el].LLambda() * vol;
            double alambda = material[el].LAlpha() * vol;
            double *K = m_elmUndeformedStiffness[el].data();
            //double K[144];
            double *BtGB = m_elmBtGB[el].data();
            double *BtHB = m_elmBtHB[el].data();
            for (int i = 0; i < 144; ++i)
                K[i] = llambda * BtGB[i] + alambda * BtHB[i];
            //double dif = 0;
            //double diff[144];
            //for (int i = 0; i < 144; ++i)
            //{
            //    diff[i] = (m_elmUndeformedStiffness[el][i] - K[i]);
            //    dif += abs(diff[i]);
            //}
            //double d = llambda*alambda;
        }
    
        ResetAllFlags();
    }

    void CorotatedLinearModel::ComputeElasticForce(const std::vector<Vector3d> &nodalPosition, std::vector<Vector3d> &nodalForce, ForceComputationMethod method /*= StressTensor*/)
    {
        PerElementOperator fn = &CorotatedLinearModel::UpdateElasticForceForElementUsingStressTensor;
        switch (method)
        {
        case StressTensor:
            break;

        case StiffneeMatrix:
            fn = &CorotatedLinearModel::UpdateElasticForceForElementUsingStiffnessMatrix;
            break;
        }
               
        UpdateAllElements(nodalPosition, &fn, 1);
    
        size_t nvert = m_topology->GetNumVertices();
        nodalForce.reserve(nvert);
        nodalForce.clear();
        nodalForce.resize(nvert, Vector3d::ZERO);

        auto &elements = m_topology->GetElements();
        size_t nelm = elements.size();
        for (size_t i = 0; i < nelm; ++i)
        {
            auto &elm = elements[i];
            auto &elmForce = m_elmElasticForces[i];
            for (int vi = 0; vi < 4; ++vi)
                nodalForce[elm[vi]] += elmForce[vi];
        }
    }
    
  
    void CorotatedLinearModel::UpdateForceStiffnessMatrix(const std::vector<Vector3d> &nodalPosition)
    {
        PerElementOperator fns[] = {
            &CorotatedLinearModel::UpdateStiffnessMatrixForElement,
            &CorotatedLinearModel::UpdateElasticForceForElementUsingStiffnessMatrix
        };

        UpdateAllElements(nodalPosition, fns, 2);
    }


    void CorotatedLinearModel::UpdateForceMaterialStiffnessMatrix(const std::vector<Vector3d> &nodalPosition)
    {
        PerElementOperator fns[] = {
            &CorotatedLinearModel::UpdateElasticForceForElementUsingStiffnessMatrix,
            &CorotatedLinearModel::UpdateMaterialStiffnessMatrixForElement
        };

        UpdateAllElements(nodalPosition, fns, 2);
    }

    void CorotatedLinearModel::UpdateLambdaAlphaDerivative(const std::vector<Vector3d> &nodalPosition)
    {
        PerElementOperator fns[] = {
            &CorotatedLinearModel::UpdateLambdaAlphaDerivativeForElement,
        };

        UpdateAllElements(nodalPosition, fns, 1);
    }

    Vector3d CorotatedLinearModel::ComputeElasticForceForNode(int nodeId, const std::vector<Vector3d> &nodalPosition, ForceComputationMethod method /*= StressTensor*/, bool reCalc /*= false*/)
    {
        Vector3d f = Vector3d::ZERO;
        auto &vneighbor = m_topology->GetVertexNeighborElements(nodeId);
        size_t nn = vneighbor.size();
        for (size_t ni = 0; ni < nn; ++ni)
        {
            auto &elm = vneighbor[ni];
            if (reCalc || !m_flagElmElasticForceCalculated[elm.elm])
            {
                switch (method)
                {
                case StressTensor:
                    UpdateElasticForceForElementUsingStressTensor(elm.elm, nodalPosition);
                    break;

                case StiffneeMatrix:
                    UpdateElasticForceForElementUsingStiffnessMatrix(elm.elm, nodalPosition);
                    break;
                }
            }

            f += m_elmElasticForces[elm.elm][elm.vidInElm];
        }

        return f;
    }

    Matrix3d CorotatedLinearModel::ComputeStiffnessMatrixForNode(int nodeId, const std::vector<Vector3d> &nodalPosition, bool reCalc /*= false*/)
    {
        Matrix3d Kmat(true);
        auto &vneighbor = m_topology->GetVertexNeighborElements(nodeId);
        size_t nn = vneighbor.size();
        for (size_t ni = 0; ni < nn; ++ni)
        {
            auto &elm = vneighbor[ni];
            if (reCalc || !m_flagElmElasticForceCalculated[elm.elm])
                UpdateStiffnessMatrixForElement(elm.elm, nodalPosition);

            double *Kelm = m_elmStiffness[elm.elm].data();
            Kelm += 12 * 3 * elm.vidInElm + 3 * elm.vidInElm;
            double *K = Kmat;
            for (int i = 0; i < 3; ++i, Kelm += 12, K += 3)
            {
                K[0] += Kelm[0]; 
                K[1] += Kelm[1];
                K[2] += Kelm[2];
            }
        }

        return Kmat;
    }

    Matrix3d CorotatedLinearModel::ComputeMaterialStiffnessMatrixForNode(int nodeId, const std::vector<Vector3d> &nodalPosition, bool reCalc /*= false*/)
    {
        Matrix3d Kmat(true);
        auto &vneighbor = m_topology->GetVertexNeighborElements(nodeId);
        size_t nn = vneighbor.size();
        for (size_t ni = 0; ni < nn; ++ni)
        {
            auto &elm = vneighbor[ni];
            if (reCalc || !m_flagElmElasticForceCalculated[elm.elm])
                UpdateMaterialStiffnessMatrixForElement(elm.elm, nodalPosition);

            double *Kelm = m_elmMaterialStiffness[elm.elm].data();
            Kelm += 12 * 3 * elm.vidInElm + 3 * elm.vidInElm;
            double *K = Kmat;
            for (int i = 0; i < 3; ++i, Kelm += 12, K += 3)
            {
                K[0] += Kelm[0]; 
                K[1] += Kelm[1];
                K[2] += Kelm[2];
            }
        }

        return Kmat;
    }
    
    bool CorotatedLinearModel::ComputeStiffnessMatrixFor(int fid, int nid, Matrix3d &stiff, bool reCalc /*= false*/)
    {
        stiff.MakeZero();
                
        auto &vne = m_topology->GetVertexNeighborElements(nid);
        size_t nn = vne.size();
        if (fid == nid)
        {            
            for (size_t ni = 0; ni < nn; ++ni)
            {
                auto &elm = vne[ni];
                double *Kelm = m_elmStiffness[elm.elm].data();
                Kelm += 12 * 3 * elm.vidInElm + 3 * elm.vidInElm;
                double *K = stiff;
                for (int i = 0; i < 3; ++i, Kelm += 12, K += 3)
                {
                    K[0] += Kelm[0]; 
                    K[1] += Kelm[1];
                    K[2] += Kelm[2];
                }
            }

            return true;
        }

        auto &vnv = m_topology->GetVertexNeighborVertices(nid);
        if (std::find(vnv.begin(), vnv.end(), fid) == vnv.end())
            return false;


        auto &elements = m_topology->GetElements();
        for (size_t ni = 0; ni < nn; ++ni)
        {
            auto &eid = vne[ni];
            auto &elm = elements[eid.elm];
            int fidInElm = std::find(elm.begin(), elm.end(), fid) - elm.begin();
            if (fidInElm == 4)
                continue;
            
            double *Kelm = m_elmStiffness[eid.elm].data();
            Kelm += 12 * 3 * fidInElm + 3 * eid.vidInElm;
            double *K = stiff;
            for (int i = 0; i < 3; ++i, Kelm += 12, K += 3)
            {
                K[0] += Kelm[0]; 
                K[1] += Kelm[1];
                K[2] += Kelm[2];
            }
        }

        return true;
    }

    bool CorotatedLinearModel::ComputeMaterialStiffnessMatrixFor(int fid, int nid, Matrix3d &stiff, bool reCalc /*= false*/)
    {
        stiff.MakeZero();

        auto &vne = m_topology->GetVertexNeighborElements(nid);
        size_t nn = vne.size();

        if (fid == nid)
        {            
            for (size_t ni = 0; ni < nn; ++ni)
            {
                auto &elm = vne[ni];
                double *Kelm = m_elmMaterialStiffness[elm.elm].data();
                Kelm += 12 * 3 * elm.vidInElm + 3 * elm.vidInElm;
                double *K = stiff;
                for (int i = 0; i < 3; ++i, Kelm += 12, K += 3)
                {
                    K[0] += Kelm[0]; 
                    K[1] += Kelm[1];
                    K[2] += Kelm[2];
                }
            }

            return true;
        }
                
        auto &vnv = m_topology->GetVertexNeighborVertices(nid);
        if (std::find(vnv.begin(), vnv.end(), fid) == vnv.end())
            return false;


        auto &elements = m_topology->GetElements();
        for (size_t ni = 0; ni < nn; ++ni)
        {
            auto &eid = vne[ni];
            auto &elm = elements[eid.elm];
            int fidInElm = std::find(elm.begin(), elm.end(), fid) - elm.begin();
            if (fidInElm == 4)
                continue;
            
            double *Kelm = m_elmMaterialStiffness[eid.elm].data();
            Kelm += 12 * 3 * fidInElm + 3 * eid.vidInElm;
            double *K = stiff;
            for (int i = 0; i < 3; ++i, Kelm += 12, K += 3)
            {
                K[0] += Kelm[0]; 
                K[1] += Kelm[1];
                K[2] += Kelm[2];
            }
        }

        return true;
    }


    const std::array<Vector3d, 4> &CorotatedLinearModel::ComputeElasticForceForElement(size_t elmId, const std::vector<Vector3d> &nodalPosition, ForceComputationMethod method /*= StressTensor*/, bool reCalc /*= false*/)
    {
        switch (method)
        {
        case StressTensor:
            return ComputeElasticForceForElementUsingStressTensor(elmId, nodalPosition, reCalc);
        case StiffneeMatrix:
            return ComputeElasticForceForElementUsingStiffnessMatrix(elmId, nodalPosition, reCalc);
        }

        return ComputeElasticForceForElementUsingStressTensor(elmId, nodalPosition, reCalc);
    }

    const std::array<Vector3d, 4> &CorotatedLinearModel::ComputeElasticForceForElementUsingStiffnessMatrix(
        size_t elmId, const std::vector<Vector3d> &nodalPosition, bool reCalc /*= false*/)
    {
        if (reCalc)
            m_flagElmElasticForceCalculated[elmId] = false;
        UpdateElasticForceForElementUsingStiffnessMatrix(elmId, nodalPosition);
        return m_elmElasticForces[elmId];
    }

    void CorotatedLinearModel::UpdateElasticForceForElementUsingStiffnessMatrix(size_t elmId, const std::vector<Vector3d> &nodalPosition)
    {
        if (m_flagElmElasticForceCalculated[elmId])
            return;

        auto &nodalForce = m_elmElasticForces[elmId];

        auto &elements = m_topology->GetElements();
        auto &elmVol = m_elmVolume;

        auto &vertIdx = elements[elmId];
        double vol = elmVol[elmId];
        // set element Vs
        auto &Vs = m_elmVs[elmId];
        for (int vi = 0; vi < 4; ++vi)
            Vs.SetColumn(vi, Vector4d(nodalPosition[vertIdx[vi]], 1.0));
    
        auto &VBm = m_elmVBm[elmId];

        // compute F
        Matrix3d &F = m_elmF[elmId];
        Matrix3d &R = m_elmR[elmId];
        Matrix3d &S = m_elmS[elmId];
        if (!m_flagFRSCalculated[elmId])
        {
            F = (Vs * m_elmVBm[elmId]).TopLeft3x3();
            DecomposeRotation(F, R, S);
            m_flagFRSCalculated[elmId] = true;
        }

        // compute f = R K (R'x - X);
        Matrix3d Rt = R.Transpose();
        auto &Vm = m_elmVm[elmId];
        double x[12];
        double Rtx_X[12];
        for (int vi = 0; vi < 4; ++vi)
        {
            auto &v = *reinterpret_cast<Vector3d *>(x + vi * 3);
            v = nodalPosition[vertIdx[vi]];
            *reinterpret_cast<Vector3d *>(Rtx_X + vi * 3) = Rt * v - Vm.GetColumnTop3(vi);
        }

        // compute Kx
        double KRtx_X[12] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
        double *K0 = m_elmUndeformedStiffness[elmId].data();
        double *K0row = K0;
        for (int xi = 0; xi < 12; ++xi, K0row += 12)
        {
            for (int xj = 0; xj < 12; ++xj)
                KRtx_X[xi] += K0row[xj] * Rtx_X[xj];
        }

        for (int vi = 0; vi < 4; ++vi)
            nodalForce[vi] = R * *reinterpret_cast<Vector3d *>(KRtx_X + vi * 3);

        m_flagElmElasticForceCalculated[elmId] = true;
    }

    const std::array<Vector3d, 4> &CorotatedLinearModel::ComputeElasticForceForElementUsingStressTensor(
        size_t elmId, const std::vector<Vector3d> &nodalPosition, bool reCalc /*= false*/)
    {
        if (reCalc)
            m_flagElmElasticForceCalculated[elmId] = false;
        UpdateElasticForceForElementUsingStressTensor(elmId, nodalPosition);
        return m_elmElasticForces[elmId];
    }

    void CorotatedLinearModel::UpdateElasticForceForElementUsingStressTensor(size_t elmId, const std::vector<Vector3d> &nodalPosition)
    {        
        if (m_flagElmElasticForceCalculated[elmId])
            return;

        auto &nodalForce = m_elmElasticForces[elmId];
    
        auto &elm = m_topology->GetElements()[elmId];

        Matrix3d Ds;
        Matrix3d P;
        Matrix3d H;
    
        Matrix3d &F = m_elmF[elmId];
        Matrix3d &R = m_elmR[elmId];
        Matrix3d &S = m_elmS[elmId];
        if (!m_flagFRSCalculated[elmId])
        {
            const Vector3d &v3 = nodalPosition[elm[3]];
            for (int i = 0; i < 3; ++i)
                Ds.SetColumn(i, nodalPosition[elm[i]] - v3);

            F = Ds * m_elmBm[elmId];
            DecomposeRotation(F, R, S);

            m_flagFRSCalculated[elmId] = true;
        }
        
        double lambda = m_elmStiffLambda[elmId];
        double mu = m_elmStiffMu[elmId];

        P = (2.0 * mu) * F + (lambda * (R.TransposeTimes(F).Trace() - 3) - 2.0 * mu) * R;

        H = P * m_elmWBmT[elmId];
                
        nodalForce[3] = Vector3d::ZERO;
        for (int i = 0; i < 3; ++i)
        {
            Vector3d fi = H.GetColumn(i);
            nodalForce[i] = fi;
            nodalForce[3] -= fi;
        }    

        m_flagElmElasticForceCalculated[elmId] = true;
    }


    const std::array<double, 12*12> &CorotatedLinearModel::ComputeStiffnessMatrixForElement(size_t elmId, const std::vector<Vector3d> &nodalPosition, bool reCalc /*= false*/)
    {
        if (reCalc)
            m_flagElmStifnessCalculated[elmId] = false;
        UpdateStiffnessMatrixForElement(elmId, nodalPosition);
        return m_elmStiffness[elmId];
    }

    const std::array<double, 12*12> &CorotatedLinearModel::ComputeMaterialStiffnessMatrixForElement(size_t elmId, const std::vector<Vector3d> &nodalPosition, bool reCalc /*= false*/)
    {
        if (reCalc)
            m_flagElmMaterialStiffnessCalculated[elmId] = false;
        UpdateMaterialStiffnessMatrixForElement(elmId, nodalPosition);
        return m_elmMaterialStiffness[elmId];
    }

    void CorotatedLinearModel::ResetStiffnessFlags(void)
    {
        std::fill(m_flagElmStifnessCalculated.begin(), m_flagElmStifnessCalculated.end(), false);
    }

    void CorotatedLinearModel::ResetMaterialStiffnessFlags(void)
    {
        std::fill(m_flagElmMaterialStiffnessCalculated.begin(), m_flagElmMaterialStiffnessCalculated.end(), false);
    }

    void CorotatedLinearModel::ResetFRSFlags(void)
    {
        std::fill(m_flagFRSCalculated.begin(), m_flagFRSCalculated.end(), false);
    }

    void CorotatedLinearModel::ResetForceFlags(void)
    {
        std::fill(m_flagElmElasticForceCalculated.begin(), m_flagElmElasticForceCalculated.end(), false);
    }

    void CorotatedLinearModel::ResetLambdaAlphaDerivativeFlags(void)
    {
        std::fill(m_flagDfDlambdaDalphaCalculated.begin(), m_flagDfDlambdaDalphaCalculated.end(), false);
    }

    void CorotatedLinearModel::ResetAllFlags(void)
    {
        ResetStiffnessFlags();
        ResetMaterialStiffnessFlags();
        ResetFRSFlags();
        ResetForceFlags();
        ResetLambdaAlphaDerivativeFlags();
    }

    // compute RK = R * K and RKRT = R * K * R^T (block-wise)
    // input: K, R
    // output: RK, RKRT
    inline static void WarpMatrix(double * K, double * R, double * RK, double * RKRT)
    {
        memset(RK, 0, sizeof(double) * 144);
        memset(RKRT, 0, sizeof(double) * 144);
        for(int i=0; i<4; i++)
            for(int j=0; j<4; j++)
            {
                // RK = R * K
                for(int k=0; k<3; k++)
                    for(int l=0; l<3; l++)
                        for(int m=0; m<3; m++)
                            RK[12 * (3 * i + k) + (3 * j + l)] += R[3 * k + m] * K[12 * (3 * i + m) + (3 * j + l)];

                // RKRT = RK * R^T
                for(int k=0; k<3; k++)
                    for(int l=0; l<3; l++)
                        for(int m=0; m<3; m++)
                            RKRT[12 * (3 * i + k) + (3 * j + l)] += RK[12 * (3 * i + k) + (3 * j + m)] * R[3 * l + m];
            }
    }

    void CorotatedLinearModel::UpdateStiffnessMatrixForElement(size_t elmId, const std::vector<Vector3d> &nodalPosition)
    {        
        if (m_flagElmStifnessCalculated[elmId])
            return;

        // compute F
        Matrix3d &F = m_elmF[elmId];
        Matrix3d &R = m_elmR[elmId];
        Matrix3d &S = m_elmS[elmId];
        if (!m_flagFRSCalculated[elmId])
        {
            auto &elements = m_topology->GetElements();
            auto &elmVol = m_elmVolume;

            auto &vertIdx = elements[elmId];
            double vol = elmVol[elmId];
            // set element Vs
            auto &Vs = m_elmVs[elmId];
            for (int vi = 0; vi < 4; ++vi)
                Vs.SetColumn(vi, Vector4d(nodalPosition[vertIdx[vi]], 1.0));
    
            auto &VBm = m_elmVBm[elmId];

            F = (Vs * m_elmVBm[elmId]).TopLeft3x3();
            DecomposeRotation(F, R, S);
            m_flagFRSCalculated[elmId] = true;
        }

        double *K = m_elmStiffness[elmId].data();
        double *K0 = m_elmUndeformedStiffness[elmId].data();

        double RK[144];
        WarpMatrix(K0, R, RK, K);

        m_flagElmStifnessCalculated[elmId] = true;
    }

    void CorotatedLinearModel::UpdateLambdaAlphaDerivativeForElement(size_t elmId, const std::vector<Vector3d> &nodalPosition)
    {
        if (m_flagDfDlambdaDalphaCalculated[elmId])
            return;
        
        double vol = m_elmVolume[elmId];
        auto &elements = m_topology->GetElements();
        auto &vertIdx = elements[elmId];

        // compute F
        Matrix3d &F = m_elmF[elmId];
        Matrix3d &R = m_elmR[elmId];
        Matrix3d &S = m_elmS[elmId];
        if (!m_flagFRSCalculated[elmId])
        {
            // set element Vs
            auto &Vs = m_elmVs[elmId];
            for (int vi = 0; vi < 4; ++vi)
                Vs.SetColumn(vi, Vector4d(nodalPosition[vertIdx[vi]], 1.0));
    
            auto &VBm = m_elmVBm[elmId];

            F = (Vs * m_elmVBm[elmId]).TopLeft3x3();
            DecomposeRotation(F, R, S);
            m_flagFRSCalculated[elmId] = true;
        }
        
        Matrix3d Rt = R.Transpose();
        auto &Vm = m_elmVm[elmId];
        double volRtx_X[12];
        for (int vi = 0; vi < 4; ++vi)
        {
            auto &v = nodalPosition[vertIdx[vi]];
            auto &v1 = *reinterpret_cast<Vector3d *>(volRtx_X + vi * 3) = Rt * v - Vm.GetColumnTop3(vi);
            v1 *= vol;
        }
        double vol_BtGB_Rtx_X[12] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
        double vol_BtHB_Rtx_X[12] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
        double *BtGBrow = m_elmBtGB[elmId].data();
        double *BtHBrow = m_elmBtHB[elmId].data();
        for (int i = 0; i < 12; ++i, BtGBrow += 12, BtHBrow += 12)
        {
            for (int j = 0; j < 12; ++j)
            {
                vol_BtGB_Rtx_X[i] += BtGBrow[j] * volRtx_X[j];
                vol_BtHB_Rtx_X[i] += BtHBrow[j] * volRtx_X[j];
            }
        }

        double *dfdl = m_elmDfDlambda[elmId].data();
        double *dfda = m_elmDfDalpha[elmId].data();
        for (int vi = 0; vi < 4; ++vi)
        {
            *reinterpret_cast<Vector3d *>(dfdl + vi * 3)
                = R * *reinterpret_cast<Vector3d *>(vol_BtGB_Rtx_X + vi * 3);

            *reinterpret_cast<Vector3d *>(dfda + vi * 3)
                = R * *reinterpret_cast<Vector3d *>(vol_BtHB_Rtx_X + vi * 3);
        }


        m_flagDfDlambdaDalphaCalculated[elmId] = true;
    }

    static Wm5::Table<12,12,double> mat12x12;

    void CorotatedLinearModel::UpdateMaterialStiffnessMatrixForElement(size_t elmId, const std::vector<Vector3d> &nodalPosition)
    {
        if (m_flagElmMaterialStiffnessCalculated[elmId])
            return;

        auto &elements = m_topology->GetElements();
        auto &elmVol = m_elmVolume;

        auto &vertIdx = elements[elmId];
        double vol = elmVol[elmId];
        // set element Vs
        auto &Vs = m_elmVs[elmId];
        for (int vi = 0; vi < 4; ++vi)
            Vs.SetColumn(vi, Vector4d(nodalPosition[vertIdx[vi]], 1.0));
    
        auto &VBm = m_elmVBm[elmId];

        // compute F
        Matrix3d &F = m_elmF[elmId];
        Matrix3d &R = m_elmR[elmId];
        Matrix3d &S = m_elmS[elmId];
        if (!m_flagFRSCalculated[elmId])
        {
            F = (Vs * m_elmVBm[elmId]).TopLeft3x3();
            DecomposeRotation(F, R, S);
            m_flagFRSCalculated[elmId] = true;
        }


        // compute dR/dF
        double sss = S.Trace();
        Matrix3d G = -S;
        G(0, 0) += sss; G(1, 1) += sss; G(2, 2) += sss;
        G = G.TimesTranspose(R);
        Matrix3d Ginv = G.Inverse();
        Matrix3d dRFij[3][3];
        Vector3d wij[3][3];
        for (int i = 0; i < 3; ++i)
        {
            for (int j = 0; j < 3; ++j)
            {
                double *m = dRFij[i][j];
            
                // copy i-th row of R into column j of temp
                double temp[9] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
                for(int k = 0; k < 3; k++)
                    temp[3 * k + j] = R(i, k);

                double skew[3] = {
                    ((temp)[7] - (temp)[5]),
                    ((temp)[2] - (temp)[6]),
                    ((temp)[3] - (temp)[1])
                };
                double *w = wij[i][j] = Ginv * *reinterpret_cast<Vector3d *>(skew);
                *(m++) =  0.0;  *(m++) = -w[2]; *(m++) =  w[1];
                *(m++) =  w[2]; *(m++) =   0.0; *(m++) = -w[0];
                *(m++) = -w[1]; *(m++) =  w[0]; *(m++) =  0.0;

                dRFij[i][j] = dRFij[i][j] * R;
            }
        }

        // x stuff
        Matrix3d Rt = R.Transpose();
        auto &Vm = m_elmVm[elmId];
        double x[12];
        double Rtx_X[12];
        for (int vi = 0; vi < 4; ++vi)
        {
            auto &v = *reinterpret_cast<Vector3d *>(x + vi * 3);
            v = nodalPosition[vertIdx[vi]];
            *reinterpret_cast<Vector3d *>(Rtx_X + vi * 3) = Rt * v - Vm.GetColumnTop3(vi);
        }

    
        // stiffness
        double *K = m_elmMaterialStiffness[elmId].data();
        double *K0 = m_elmUndeformedStiffness[elmId].data();
        double *volBtE = m_elmVolBtE[elmId].data();
        memset(K, 0, sizeof(double) * 144);
        double *Kcol = K;
        for (int kl = 0; kl < 12; ++kl, ++Kcol)
        {
            int k = kl % 3;
            int l = kl / 3;
            // d(Vol) / d(Vm)_kl
            // d |Vm|/ dVm = |Vm| Vm^{-T} = |Vm| VBm^T
            //double volVkl = vol * VBm(l, k);
            double volVkl_vol = VBm(l, k);
        
            // d(VBm)_ij / d(Vm)_kl
            Matrix4d bmVkl;// = *reinterpret_cast<Matrix4d *>(alloca(sizeof(Matrix4d)));
            for (int i = 0; i < 4; ++i)
                for (int j = 0; j < 4; ++j)
                    bmVkl(i, j) = -VBm(i, k) * VBm(l, j);
        
            // d(F)_ij / d(Vm)_kl
            //Matrix3d &fvkl = *reinterpret_cast<Matrix3d *>(alloca(sizeof(Matrix3d)));
            Matrix3d fVkl = (Vs * bmVkl).TopLeft3x3();
            // d(R)_ij / d(Vm)_kl
            Matrix3d rVkl(true) ;//= *reinterpret_cast<Matrix3d *>(alloca(sizeof(Matrix3d)));
            for (int i = 0; i < 3; ++i)
            {
                for (int j = 0; j < 3; ++j)
                    rVkl += dRFij[i][j] * fVkl(i, j);
            }
            Matrix3d rVklt = rVkl.Transpose();

            // compute K_kl = d(f) / d(Vm)_kl
            // Term 1: dR/d(Vm)_kl * K (Rtx - X)
            double term1[12] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
            double KRtx_X[12] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
            double *K0row = K0;
            for (int xi = 0; xi < 12; ++xi, K0row += 12)
            {
                for (int xj = 0; xj < 12; ++xj)
                    KRtx_X[xi] += K0row[xj] * Rtx_X[xj];
            }
            for (int vi = 0; vi < 4; ++vi)
            {
                *reinterpret_cast<Vector3d *>(term1 + vi * 3) = 
                    rVkl * *reinterpret_cast<Vector3d *>(KRtx_X + vi * 3);
            }

            // Term 2: RK( d(R)/d(Vm)_kl * x - d(X) / d(Vm)_kl )
            double term2[12] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
            double dRtx_X[12];
            for (int vi = 0; vi < 4; ++vi)
            {
                auto &v = *reinterpret_cast<Vector3d *>(x + vi * 3);
                auto &tv = *reinterpret_cast<Vector3d *>(dRtx_X + vi * 3);
                tv = rVklt * v;
            }
            dRtx_X[kl] -= 1.0;
            K0row = K0;
            for (int xi = 0; xi < 12; ++xi, K0row += 12)
            {
                for (int xj = 0; xj < 12; ++xj)
                    term2[xi] += K0row[xj] * dRtx_X[xj];
            }
            for (int vi = 0; vi < 4; ++vi)
            {
                auto &v = *reinterpret_cast<Vector3d *>(term2 + vi * 3);
                v = R * v;
            }

            // Term 3: R (d(K)/d(Vm)_kl ) (Rtx - X)
            // ***** K = VBtEB
            // Term 3.1 (d(V) / V) K (Rtx - X);
            double term31[12] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
            for (int vi = 0; vi < 12; ++vi)
            {
                term31[vi] = KRtx_X[vi] * volVkl_vol;
            }

            // Term 3.2: [d(B)/d(Vm)_kl]t (VBtE)t (Rtx - X)
            double term32[12] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
            double vec6[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
            double *volBtErow = volBtE;
            for (int vj = 0; vj < 12; ++vj, volBtErow += 6)
            {
                for (int vi = 0; vi < 6; ++vi)
                    vec6[vi] += volBtErow[vi] * Rtx_X[vj];
            }
            //PrintMatrix(volBtE, 12, 6, "%20.15g ");
            BTransposeTimesMatrix(bmVkl, vec6, 1, term32);

            // Term 3.3: (VBtE) [d(B)/d(Vm)_kl] (Rtx - X)
            double term33[12] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
            BTimesMatrix(bmVkl, Rtx_X, 1, vec6);
            volBtErow = volBtE;
            for (int vi = 0; vi < 12; ++vi, volBtErow += 6)
            {
                for (int vj = 0; vj < 6; ++vj)
                    term33[vi] += volBtErow[vj] * vec6[vj];
            }
            // Term 3: R (terms)        
            double term3[12] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
            for (int vi = 0; vi < 12; ++vi)
                term3[vi] = term31[vi] + term32[vi] + term33[vi];
            for (int vi = 0; vi < 4; ++vi)
            {
                auto &v = *reinterpret_cast<Vector3d *>(term3 + vi * 3);
                v = R * v;
            }

            /////////////////////////////////////
            // sums up
            for (int rowi = 0; rowi < 12; ++rowi)
            {
                Kcol[rowi * 12] = (term1[rowi] + term2[rowi] + term3[rowi]);
            }
        }
    
        //printf("K\n");
        //PrintMatrix(K, 12, 12, "%20.15g ");

        m_flagElmMaterialStiffnessCalculated[elmId] = true;
    }

    void CorotatedLinearModel::UpdateAllElements(const std::vector<Vector3d> &nodalPosition, PerElementOperator fns[], int n)
    {
        for (int i = 0; i < n; ++i)
        {
            size_t nelm = m_topology->GetElements().size();
            for (size_t elmId = 0; elmId < nelm; ++elmId)
                (this->*fns[i])(elmId, nodalPosition);
        }
}

}