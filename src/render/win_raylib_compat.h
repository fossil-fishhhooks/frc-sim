#pragma once
// win_raylib_compat.h — injected via /FI into every translation unit on MSVC.
//
// Problem: wingdi.h declares  int Rectangle(HDC, int, int, int, int)
//          winuser.h declares BOOL CloseWindow(HWND), int ShowCursor(BOOL)
//          winuser.h #defines DrawText   → DrawTextA
//          winuser.h #defines DrawTextEx → DrawTextExA
//          winuser.h #defines LoadImage  → LoadImageA
//          playsoundapi.h #defines PlaySound → PlaySoundA
// All of these collide with raylib's identically-named API symbols.
//
// Fix: parse raylib.h FIRST so the compiler sees raylib's struct/function
// declarations before windows.h contaminates the global namespace.
// Then, before including windows.h, redefine the conflicting *function*
// names as macros so winuser.h declares them under mangled names instead.
// After windows.h, undef both the macro aliases and the macro-only collisions.
//
// This header is only active on Windows; on other platforms it is a no-op.

#ifdef _WIN32

// Step 1 — raylib first, before any Windows header gets a chance to run.
#include <raylib.h>

// Step 2 — rename Win32 functions that clash with raylib *before* windows.h
// sees them. winuser.h guards its declarations with these macros, so this
// causes the Win32 prototypes to be emitted under the mangled names and
// prevents the extern-"C"-linkage redefinition errors.
#define CloseWindow  _Win32_CloseWindow
#define ShowCursor   _Win32_ShowCursor

// Step 3 — slim Windows headers.
#ifndef WIN32_LEAN_AND_MEAN
#  define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#  define NOMINMAX
#endif
#include <winsock2.h>   // must precede windows.h
#include <ws2tcpip.h>
#include <windows.h>

// Step 4 — drop all the aliases so downstream code sees raylib's names cleanly.
#undef CloseWindow
#undef ShowCursor

// Step 5 — strip macro-only collisions Windows defined on top of raylib's names.
#ifdef DrawText
#  undef DrawText
#endif
#ifdef DrawTextEx
#  undef DrawTextEx
#endif
#ifdef LoadImage
#  undef LoadImage
#endif
#ifdef PlaySound
#  undef PlaySound
#endif
#ifdef CreateWindow
#  undef CreateWindow
#endif
#ifdef GetObject
#  undef GetObject
#endif
#ifdef GetMessage
#  undef GetMessage
#endif

#endif // _WIN32