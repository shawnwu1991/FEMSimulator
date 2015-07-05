#pragma once

#include "CorotatedLinearModel.h"

#include <QtCore\QThreadPool>

namespace Libin
{
    class CorotatedLinearModelMultiThreaded : public CorotatedLinearModel
    {
    public:
        CorotatedLinearModelMultiThreaded(std::shared_ptr<TetMeshTopology> topology, int numThread = 0);
        ~CorotatedLinearModelMultiThreaded(void);
        
    protected:
        virtual void UpdateAllElements(const std::vector<Vector3d> &nodalPosition, PerElementOperator fns[], int n);

        class ThreadTask : public QRunnable
        {
        public:
            void run(void);

            int startElmId;
            int endElmId;

            CorotatedLinearModel *model;
            PerElementOperator fn;
            const std::vector<Vector3d> *pNodalPosition;
        };

    private:        
        int m_numThread;
        int m_elmPerJob;
        QThreadPool m_threadPool;
    };
}

