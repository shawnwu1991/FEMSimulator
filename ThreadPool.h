#pragma once

#ifndef _CORE_THREAD_POOL_H_
#define _CORE_THREAD_POOL_H_

#include <vector>
#include <Windows.h>

namespace SIMCON
{
    class CThreadPool
    {
    public:
        static CThreadPool &Instance(int id = 0, int initNum = -1);

    public:
        // run a task, return the thread id
        // return -1 if any error occurs
        int RunTask(void (*fnTask)(void *), void *pArg, bool createNew = true);

        // wait until a task finishes
        // return 0 if the task finishes before timeout
        // return -1 if timeout
        // return -2 if any error occurs
        int WaitForTask(int id, int timeout);

        // wait for multiple tasks
        // return the task id if it finishes before timeout
        // return -1 if timeout
        // return -2 if any error occurs
        // if n == 0 this function will wait on all tasks
        int WaitForTasks(int n, int *ids, bool waitAll, int timeout);

    protected:        
        static DWORD WINAPI ThreadPoolWorker(LPVOID param);
        void CreateNewThread(void);

    protected:
        int m_nMinNumThread;
        struct ThreadInfo
        {
            HANDLE hThread;
            HANDLE hTaskNewEvent;
            HANDLE hTaskDoneEvent;

            void (*fnTask)(void *);
            void *pArg;
        };

        int m_nextTaskId;
        std::vector<ThreadInfo *> m_vThreads;
        
    public:
        CThreadPool(int nMin);
        ~CThreadPool(void);
    };
}

#endif