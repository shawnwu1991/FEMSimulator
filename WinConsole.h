#pragma once

#include <windows.h>

class CWinConsole
{
protected:
    CWinConsole(void);
    ~CWinConsole(void);

public:
    static BOOL CreateWinConsole(void);
    static BOOL FreeWinConsole(void);
    static CWinConsole *GetConsole(void);

    static BOOL SetTitle(const TCHAR *name);

    static HWND GetConsoleHandle(void);

    static DWORD SetConsoleFontSize(DWORD nFont);

protected:
    static CWinConsole *ms_pConsole;
    
    static HWND m_hWindow;
    static HANDLE m_hStdIn;
    static HANDLE m_hStdOut;
    static HANDLE m_hStdErr;
};

class ScopedWinConsole
{
public:
    ScopedWinConsole(void) { CWinConsole::CreateWinConsole(); }
    ~ScopedWinConsole(void) { CWinConsole::FreeWinConsole(); }
};