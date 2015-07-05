#include "stdafx.h"
#include "CorotatedLinearModelMultiThreaded.h"

namespace Libin
{
    CorotatedLinearModelMultiThreaded::CorotatedLinearModelMultiThreaded(std::shared_ptr<TetMeshTopology> topology, int numThread /*= 1*/)
        : CorotatedLinearModel(topology)
        , m_numThread(numThread)
        , m_elmPerJob(40)
    {
        if (numThread)
            m_threadPool.setMaxThreadCount(numThread);
    }


    CorotatedLinearModelMultiThreaded::~CorotatedLinearModelMultiThreaded(void)
    {
    }

    void CorotatedLinearModelMultiThreaded::UpdateAllElements(const std::vector<Vector3d> &nodalPosition, PerElementOperator fns[], int n)
    {
        int elm = GetTopology()->GetNumElement();
        int ntaskblocks = elm / m_elmPerJob + ((elm % m_elmPerJob) ? 1 : 0);
        std::vector<ThreadTask> taskPool(n * ntaskblocks);
        auto it = taskPool.begin();
        for (int i = 0; i < n; ++i)
        {
            int start = 0;
            for (size_t bi = ntaskblocks - 1; bi > 0; --bi, ++it)
            {
                auto &task = *it;
                task.pNodalPosition = &nodalPosition;
                task.model = this;
                task.startElmId = start;
                task.endElmId = (start += m_elmPerJob);
                task.fn = fns[i];

                task.setAutoDelete(false);
            }
            
            auto &task = *(it++);
            task.pNodalPosition = &nodalPosition;
            task.model = this;
            task.startElmId = start;
            task.endElmId = elm;
            task.fn = fns[i];

            task.setAutoDelete(false);
        }

        for (auto &task : taskPool)
            m_threadPool.start(&task);

        m_threadPool.waitForDone();
    }

    void CorotatedLinearModelMultiThreaded::ThreadTask::run(void)
    {
        for (int i = startElmId; i < endElmId; ++i)
        {
            (model->*fn)(i, *pNodalPosition);
        }
    }
}
