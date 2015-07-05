#include "stdafx.h"
#include "ThreadPool.h"
#include <memory>
#include <algorithm>
#include <map>

namespace SIMCON
{    
    static void GetAndPrintError(void)
    {
        TCHAR buf[1024];
        DWORD err = GetLastError();
        DWORD rt = FormatMessage(FORMAT_MESSAGE_FROM_SYSTEM, NULL, err, LANG_USER_DEFAULT, buf, 1024, NULL);
        if (!rt)
        {
            printf("Fail to format message: %d\n", rt);
            return;
        }

        buf[rt] = 0;
        printf("Error! 0x%08x: %s\n", err, buf);
    }
    
    CThreadPool::CThreadPool(int nMin)
        : m_nextTaskId(1)
    {
        for (int i = 0; i < nMin; ++i)
            CreateNewThread();
    }

    CThreadPool::~CThreadPool(void)
    {
        for (size_t i = 0; i < m_vThreads.size(); ++i)
        {            
            if (WAIT_FAILED == ::WaitForSingleObject(m_vThreads[i]->hTaskDoneEvent, 1))
            {
                GetAndPrintError();
                continue;
            }
            
            if (!::ResetEvent(m_vThreads[i]->hTaskDoneEvent))
            {
                GetAndPrintError();
                continue;
            }   

            m_vThreads[i]->fnTask = NULL;

            if (!::SetEvent(m_vThreads[i]->hTaskNewEvent))
            {
                GetAndPrintError();
                continue;
            }   

            if (WAIT_FAILED == ::WaitForSingleObject(m_vThreads[i]->hTaskDoneEvent, 1))
            {
                GetAndPrintError();
                continue;
            }
        }

        
        for (size_t i = 0; i < m_vThreads.size(); ++i)
        {
            if (!::CloseHandle(m_vThreads[i]->hThread))
                GetAndPrintError();
            if (!::CloseHandle(m_vThreads[i]->hTaskDoneEvent))
                GetAndPrintError();
            if (!::CloseHandle(m_vThreads[i]->hTaskNewEvent))
                GetAndPrintError();

            delete m_vThreads[i];
        }
    }
    
    // run a task, return the thread id
    int CThreadPool::RunTask(void (*fnTask)(void *), void *pArg, bool createNew /*= true*/)
    {
        if (!fnTask)
            return -1;
        
        for (size_t i = 0; i < m_vThreads.size(); ++i)
        {
            if (m_vThreads[i]->fnTask == NULL)
            {                
                DWORD dw = ::WaitForSingleObject(m_vThreads[i]->hTaskDoneEvent, 0);
                if (dw == WAIT_FAILED)
                {
                    GetAndPrintError();
                    continue;
                }
                if (dw == WAIT_TIMEOUT)
                {
                    continue;
                }

                m_vThreads[i]->fnTask = fnTask;
                m_vThreads[i]->pArg = pArg;
                if (!::ResetEvent(m_vThreads[i]->hTaskDoneEvent))
                {
                    GetAndPrintError();
                    m_vThreads[i]->fnTask = NULL;
                    continue;
                }
                if (!::SetEvent(m_vThreads[i]->hTaskNewEvent))
                {
                    GetAndPrintError();
                    m_vThreads[i]->fnTask = NULL;
                    continue;
                }

                return i;
            }
        }

        if (!createNew)
            return -1;

        int id = m_vThreads.size();
        CreateNewThread();
        if (id == m_vThreads.size())
            return -1;
                
        m_vThreads[id]->fnTask = fnTask;
        m_vThreads[id]->pArg = pArg;
        if (!::SetEvent(m_vThreads[id]->hTaskNewEvent))
        {
            GetAndPrintError();
            m_vThreads[id]->fnTask = NULL;
            return -1;
        }

        return id;
    }

    // wait until a task finishes
    // return 0 if the task finishes before timeout
    // return -1 if timeout
    // return -2 if any error occurs
    int CThreadPool::WaitForTask(int id, int timeout)
    {
        if (id < 0 || id >= (int)m_vThreads.size())
            return -2;

        DWORD dw = ::WaitForSingleObject(m_vThreads[id]->hTaskDoneEvent, timeout);
        if (dw == WAIT_TIMEOUT)
            return -1;
        if (dw == WAIT_FAILED)
        {
            GetAndPrintError();
            return -2;
        }

        return 0;
    }

    // wait for multiple tasks
    // return the task id if it finishes before timeout
    // return -1 if timeout
    // return -2 if any error occurs
    // if n == 0 this function will wait on all tasks
    int CThreadPool::WaitForTasks(int n, int *ids, bool waitAll, int timeout)
    {
        if (n < 0)
            return -2;
        if (n > 0 && !ids)
            return -2;
        
        HANDLE *handles = new HANDLE[n == 0 ? m_vThreads.size() : n];
        if (n == 0)
        {
            HANDLE *p = handles;
            for each (auto &t in m_vThreads)
                *p++ = t->hTaskDoneEvent;
            n = m_vThreads.size();
        }
        else
        {
            for (int i = 0; i < n; ++i)
                handles[i] = m_vThreads[ids[i]]->hTaskDoneEvent;
            n = std::unique(handles, handles + n) - handles;
        }

        DWORD dw = ::WaitForMultipleObjects(n, handles, waitAll ? TRUE : FALSE, timeout);
        if (dw == WAIT_TIMEOUT)
        {
            delete handles;
            return -1;
        }
        if (dw == WAIT_FAILED)
        {
            GetAndPrintError();
            delete handles;
            return -2;
        }
        delete handles;
        //return dw - WAIT_OBJECT_0;
        if (ids && n > 0)
            return ids[dw - WAIT_OBJECT_0];
        else
            return dw - WAIT_OBJECT_0;
    }

    DWORD WINAPI CThreadPool::ThreadPoolWorker(LPVOID param)
    {
        ThreadInfo &info = *(ThreadInfo *)param;
        if (!::SetEvent(info.hTaskDoneEvent))
        {            
            GetAndPrintError();
            return -1;
        }
        
        while (true)
        {
            if (WAIT_FAILED == ::WaitForSingleObject(info.hTaskNewEvent, INFINITE))
            {
                GetAndPrintError();
                return -1;
            }
            if (!::ResetEvent(info.hTaskNewEvent))
            {
                GetAndPrintError();
                return -1;
            }
            if (!::ResetEvent(info.hTaskDoneEvent))
            {
                GetAndPrintError();
                return -1;
            }

            if (!info.fnTask)
                break;

            info.fnTask(info.pArg);
            info.fnTask = NULL;

            if (!::SetEvent(info.hTaskDoneEvent))
            {
                GetAndPrintError();
                return -1;
            }
        }

        if (!::SetEvent(info.hTaskDoneEvent))
        {
            GetAndPrintError();
            return -1;
        }
        return 0;
    }

    void CThreadPool::CreateNewThread(void)
    {
        ThreadInfo &info = *(new ThreadInfo);
        info.fnTask = NULL;
        info.hThread = ::CreateThread(NULL, 0, ThreadPoolWorker, &info, CREATE_SUSPENDED, NULL);
        if (info.hThread == NULL)
        {
            GetAndPrintError();
            delete &info;
            return;
        }

        if (!(info.hTaskDoneEvent = ::CreateEvent(NULL, TRUE, FALSE, NULL)))
        {
            GetAndPrintError();
            if (!::CloseHandle(info.hThread))
                GetAndPrintError();
            delete &info;
            return;
        }
        
        if (!(info.hTaskNewEvent = ::CreateEvent(NULL, TRUE, FALSE, NULL)))
        {
            GetAndPrintError();
            if (!::CloseHandle(info.hThread))
                GetAndPrintError();
            if (!::CloseHandle(info.hTaskDoneEvent))
                GetAndPrintError();
            delete &info;
            return;
        }

        if (!::ResumeThread(info.hThread))
        {
            GetAndPrintError();
            if (!::CloseHandle(info.hThread))
                GetAndPrintError();
            if (!::CloseHandle(info.hTaskDoneEvent))
                GetAndPrintError();
            if (!::CloseHandle(info.hTaskNewEvent))
                GetAndPrintError();
            delete &info;
            return;
        }

        if (WAIT_FAILED == ::WaitForSingleObject(info.hTaskDoneEvent, INFINITE))
        {
            GetAndPrintError();
            if (!::CloseHandle(info.hThread))
                GetAndPrintError();
            if (!::CloseHandle(info.hTaskDoneEvent))
                GetAndPrintError();
            if (!::CloseHandle(info.hTaskNewEvent))
                GetAndPrintError();
            delete &info;
            return;
        }

        m_vThreads.push_back(&info);
    }
    
    CThreadPool &CThreadPool::Instance(int id /*= 0*/, int initNum /*= -1*/)
    {
        //static CThreadPool s_instance(8);
        static std::map<int, std::shared_ptr<CThreadPool>> s_instances;

        auto it = s_instances.find(id);
        if (it != s_instances.end())
            return *it->second;

        if (initNum < 0)
        {
            SYSTEM_INFO si;
            ::GetSystemInfo(&si);
            initNum = si.dwNumberOfProcessors;
        }
        
        s_instances[id] = std::make_shared<CThreadPool>(initNum);
        //if (!ms_pInstance)
        //{
        //    int nMin = 4;
        //    SYSTEM_INFO info;
        //    ::GetSystemInfo(&info);
        //    nMin = std::max<int>(info.dwNumberOfProcessors, nMin);
        //    ms_pInstance.reset(new CThreadPool(nMin));
        //}

        return *s_instances[id];
    }
}