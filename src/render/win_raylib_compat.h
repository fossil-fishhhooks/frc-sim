#pragma once
// win_raylib_compat.h
// Include this BEFORE raylib.h in any translation unit on Windows.
//
// Windows SDK (winuser.h, wingdi.h, playsoundapi.h) defines several
// names that collide with raylib's API.  The canonical fix is:
//   1. Include windows.h first (with slimming macros).
//   2. Immediately #undef every colliding name.
//
// On non-Windows this header is a no-op.

#ifdef _WIN32

// Slim down windows.h as much as possible.
#  ifndef WIN32_LEAN_AND_MEAN
#    define WIN32_LEAN_AND_MEAN
#  endif
#  ifndef NOMINMAX
#    define NOMINMAX
#  endif
#  ifndef VC_EXTRA_LEAN
#    define VC_EXTRA_LEAN
#  endif

// Winsock must come before windows.h to avoid winsock/winsock2 conflicts.
// wpilib and our StreamEncoder both need winsock2, so pull it in here once.
#  include <winsock2.h>
#  include <ws2tcpip.h>
#  include <windows.h>

// ── Undefine every Win32 symbol that collides with raylib ────────────────────

// wingdi.h: Rectangle is a GDI function (wingdi.h line ~4639)
#  ifdef Rectangle
#    undef Rectangle
#  endif

// winuser.h: these are #define macros that expand to the A/W variants
#  ifdef CloseWindow
#    undef CloseWindow
#  endif
#  ifdef ShowCursor
#    undef ShowCursor
#  endif
#  ifdef DrawText
#    undef DrawText
#  endif
#  ifdef DrawTextEx
#    undef DrawTextEx
#  endif
#  ifdef LoadImage
#    undef LoadImage
#  endif

// playsoundapi.h
#  ifdef PlaySound
#    undef PlaySound
#  endif

// A few others that have been known to collide in some SDK versions
#  ifdef CreateWindow
#    undef CreateWindow
#  endif
#  ifdef GetObject
#    undef GetObject
#  endif
#  ifdef GetMessage
#    undef GetMessage
#  endif
#  ifdef MessageBox
#    undef MessageBox
#  endif

#endif // _WIN32