#pragma once
// win_raylib_compat.h — injected via /FI into every translation unit on MSVC.
//
// Problem: wingdi.h declares  int Rectangle(HDC, int, int, int, int)
//          winuser.h declares CloseWindow(), ShowCursor()
//          winuser.h #defines DrawText   → DrawTextA
//          winuser.h #defines DrawTextEx → DrawTextExA
//          winuser.h #defines LoadImage  → LoadImageA
//          playsoundapi.h #defines PlaySound → PlaySoundA
// All of these collide with raylib's identically-named API symbols.
//
// Fix: parse raylib.h FIRST so the compiler sees raylib's struct/function
// declarations before windows.h contaminates the global namespace.
// Then include windows.h and immediately undef the macros it layered on top.
//
// This header is only active on Windows; on other platforms it is a no-op.

#ifdef _WIN32

// Step 1 — raylib first, before any Windows header gets a chance to run.
#include <raylib.h>

// Step 2 — slim Windows headers.
#ifndef WIN32_LEAN_AND_MEAN
#  define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#  define NOMINMAX
#endif
#include <winsock2.h>   // must precede windows.h
#include <ws2tcpip.h>
#include <windows.h>

// Step 3 — strip the macros Windows defined on top of raylib's names.
// Rectangle in wingdi.h is a real function (not a macro) — including
// raylib.h first already handled that one.  The rest below are macros.
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
#ifdef CloseWindow
#  undef CloseWindow
#endif
#ifdef ShowCursor
#  undef ShowCursor
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