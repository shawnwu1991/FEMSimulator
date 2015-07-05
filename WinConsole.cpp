#include "StdAfx.h"
#include "comHeader.h"
#include "WinConsole.h"
#include <time.h>
#include <tchar.h>

//struct CONSOLE_FONT_INFO
//{
//    DWORD index;
//    COORD dim;
//};
//typedef CONSOLE_FONT_INFO *PCONSOLE_FONT_INFO;

typedef BOOL (WINAPI *PROCSETCONSOLEFONT)(HANDLE, DWORD);
typedef BOOL (WINAPI *PROCGETCONSOLEFONTINFO)(HANDLE,BOOL,DWORD,PCONSOLE_FONT_INFO);
//typedef COORD (WINAPI *PROCGETCONSOLEFONTSIZE)(HANDLE,DWORD);
typedef DWORD (WINAPI *PROCGETNUMBEROFCONSOLEFONTS)();
//typedef BOOL (WINAPI *PROCGETCURRENTCONSOLEFONT)(HANDLE,BOOL,PCONSOLE_FONT_INFO);

PROCSETCONSOLEFONT SetConsoleFont;
PROCGETCONSOLEFONTINFO GetConsoleFontInfo;
//PROCGETCONSOLEFONTSIZE GetConsoleFontSize;
PROCGETNUMBEROFCONSOLEFONTS GetNumberOfConsoleFonts;
//PROCGETCURRENTCONSOLEFONT GetCurrentConsoleFont;


CWinConsole *CWinConsole::ms_pConsole = NULL;
HWND CWinConsole::m_hWindow = NULL;
HANDLE CWinConsole::m_hStdIn = NULL;
HANDLE CWinConsole::m_hStdOut = NULL;
HANDLE CWinConsole::m_hStdErr = NULL;

CWinConsole::CWinConsole(void)
{
}

CWinConsole::~CWinConsole(void)
{
}

BOOL CWinConsole::CreateWinConsole()
{
    if (ms_pConsole)
    {
        return FALSE;
    }

    HMODULE hKernel32 = GetModuleHandle(TEXT("kernel32"));
    SetConsoleFont = (PROCSETCONSOLEFONT)GetProcAddress(hKernel32, ("SetConsoleFont"));
    GetConsoleFontInfo = (PROCGETCONSOLEFONTINFO)GetProcAddress(hKernel32,("GetConsoleFontInfo"));
    //GetConsoleFontSize = (PROCGETCONSOLEFONTSIZE)GetProcAddress(hKernel32,"GetConsoleFontSize");
    GetNumberOfConsoleFonts = (PROCGETNUMBEROFCONSOLEFONTS)GetProcAddress(hKernel32,"GetNumberOfConsoleFonts");
    //GetCurrentConsoleFont = (PROCGETCURRENTCONSOLEFONT)GetProcAddress(hKernel32,"GetCurrentConsoleFont");

    ms_pConsole = new CWinConsole;

    // alloc console
    if (!AllocConsole())
    {
        delete ms_pConsole;
        ms_pConsole = NULL;
        return FALSE;
    }

    // parse handles
    ms_pConsole->m_hStdIn = GetStdHandle(STD_INPUT_HANDLE);
    ms_pConsole->m_hStdOut = GetStdHandle(STD_OUTPUT_HANDLE);
    ms_pConsole->m_hStdErr = GetStdHandle(STD_ERROR_HANDLE);

    if (ms_pConsole->m_hStdIn == INVALID_HANDLE_VALUE ||
        ms_pConsole->m_hStdErr == INVALID_HANDLE_VALUE ||
        ms_pConsole->m_hStdOut == INVALID_HANDLE_VALUE
        )
    {
        delete ms_pConsole;
        ms_pConsole = NULL;
        FreeConsole();
        return FALSE;
    }

    DWORD dwInMode = ENABLE_ECHO_INPUT | ENABLE_INSERT_MODE | ENABLE_LINE_INPUT 
        | ENABLE_PROCESSED_INPUT | ENABLE_QUICK_EDIT_MODE | ENABLE_WINDOW_INPUT | ENABLE_EXTENDED_FLAGS;

    DWORD dwOutMode = ENABLE_PROCESSED_OUTPUT | ENABLE_WRAP_AT_EOL_OUTPUT;
    // redirect io
    SetConsoleMode(ms_pConsole->m_hStdIn, dwInMode);
    SetConsoleMode(ms_pConsole->m_hStdOut, dwOutMode);
    SetConsoleMode(ms_pConsole->m_hStdErr, dwOutMode);
    
    freopen("CONOUT$", "w", stdout);
    freopen("CONIN$", "r", stdin);
    freopen("CONOUT$", "w", stderr);


    CONSOLE_SCREEN_BUFFER_INFO bInfo; 
    GetConsoleScreenBufferInfo(ms_pConsole->m_hStdOut, &bInfo);
    bInfo.dwSize.X = 256;
    SetConsoleScreenBufferSize(ms_pConsole->m_hStdOut, bInfo.dwSize);

    // get window handle
    int iGet = 0;
    TCHAR buf[100];
    while (ms_pConsole->m_hWindow == NULL && iGet < 10)
    {
        srand((unsigned int)time(NULL));
#if UNICODE
        swprintf_s(buf, 100, L"__Simfr_a_b_%dd_%d_%d__", rand(), rand(), rand());
#else
        sprintf_s(buf, 100, "__Simfr_a_b_%dd_%d_%d__", rand(), rand(), rand());
#endif
        ::SetConsoleTitle(buf);
        ms_pConsole->m_hWindow = ::FindWindow(NULL, buf);

        if (ms_pConsole->m_hWindow == NULL)
        {
            ::Sleep(60);
        }
        iGet++;
    };

    if (ms_pConsole->m_hWindow == NULL)
    {
        printf("Warning! Failed to get console handle!\n");
    }
    else
    {
        // disable close button
        HMENU hMenu = ::GetSystemMenu(ms_pConsole->m_hWindow, false);
        ::RemoveMenu(hMenu, SC_CLOSE, MF_BYCOMMAND);
    }

    ::SetConsoleTitle(TEXT("Console"));


    return TRUE;
}

BOOL CWinConsole::FreeWinConsole()
{
    if (!ms_pConsole)
    {
        return FALSE;
    }

    delete ms_pConsole;
    ms_pConsole = NULL;

    FreeConsole();

    return TRUE;
}

CWinConsole *CWinConsole::GetConsole()
{
    return ms_pConsole;
}

BOOL CWinConsole::SetTitle(const TCHAR *name)
{
    return SetConsoleTitle(name);
}

HWND CWinConsole::GetConsoleHandle(void)
{
    return m_hWindow;
}

DWORD CWinConsole::SetConsoleFontSize(DWORD nFont)
{
    if (!ms_pConsole)
        return (DWORD)-1;

    // number of console fonts
    //const DWORD MAX_FONTS = 40;
    DWORD num_fonts = GetNumberOfConsoleFonts();
    //printf("Total Font: %d\n", num_fonts);
    //if (num_fonts > MAX_FONTS)
    //    num_fonts = MAX_FONTS;

    if (nFont >= num_fonts)
        return num_fonts;

    //CONSOLE_FONT_INFO fonts[MAX_FONTS] = {0};
    //GetConsoleFontInfo(ms_pConsole->m_hStdOut, 0, num_fonts, fonts);

    //for (DWORD n = 0; n < num_fonts; ++n)
    //    fonts[n].dwFontSize = GetConsoleFontSize(ms_pConsole->m_hStdOut, fonts[n].nFont);

    SetConsoleFont(ms_pConsole->m_hStdOut, nFont);


    return nFont;
}
