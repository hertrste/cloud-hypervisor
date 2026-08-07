# RAMFB VNC Issues Tracker

## Rules
- **Use the TigerVNC agent (`explore` or `task` with tigervnc context) for all TigerVNC code questions**
- Do NOT modify TigerVNC source code; consider it correct and adapt the server
- Keep this file updated with all findings

## Status: RESOLVED ✓

## Problem Statement
TigerVNC vncviewer reported "Invalid pixel format" when connecting to cloud-hypervisor VNC server.

## Root Cause: RFB Protocol Type Size Mismatch

The server sent RFB protocol fields with wrong type sizes:

### Bug 1: ServerInit width/height as CARD32 instead of CARD16
- **File:** `display/src/vnc.rs`, `server_init()` function
- **Bug:** `width.to_be_bytes()` sends 4 bytes (CARD32)
- **RFB spec:** width and height are CARD16 (2 bytes each)
- **Effect:** Client read width=0 (first 2 bytes of 4-byte value), height=high-word of width value
- **Fix:** `(width as u16).to_be_bytes()` - send as 2 bytes

### Bug 2: FramebufferUpdate rectangle width/height as CARD32 instead of CARD16
- **File:** `display/src/vnc.rs`, `send_framebuffer_update()` function
- **Bug:** Extra 2 bytes padding + width/height as CARD32 (4 bytes each)
- **RFB spec:** x, y, width, height are all CARD16 (2 bytes each), encoding is CARD32 (4 bytes)
- **Effect:** Client saw "Unknown encoding" because encoding field was misaligned
- **Fix:** Removed extra padding, used CARD16 for width/height

### Pixel Format (was correct)
- TigerVNC reads: bpp(CARD8), depth(CARD8), bigEndian(CARD8), trueColour(CARD8), redMax(CARD16), greenMax(CARD16), blueMax(CARD16), redShift(CARD8), greenShift(CARD8), blueShift(CARD8), padding(3 bytes)
- Total: 16 bytes - server implementation was correct

## TigerVNC Event Loop Investigation
Used TigerVNC agent to trace whether `processSecurityResultMsg()` could be called twice.
**Conclusion:** Cannot be called twice - state machine changes state unconditionally after each call. The double log observed earlier was a logging artifact (multiple log handlers).

## Key Learnings
1. RFB protocol uses CARD16 for most fields (width, height, x, y), not CARD32
2. Only security result, name length, and encoding are CARD32
3. Pixel format is 16 bytes with mixed CARD8/CARD16 fields
4. TigerVNC agent (`explore` subagent type) is essential for understanding client behavior

## Relevant Files
- `display/src/vnc.rs` - VNC server implementation (handshake, security, server_init, framebuffer updates)
- `display/src/framebuffer.rs` - Framebuffer abstraction
- `display/src/ramfb.rs` - RAMFB device discovery via fw_cfg

## VM Auto-Shutdown Issue
VM shuts down after ~15-20s ("VM exit event" → shutdown). Likely cloud image detecting no console activity.
- `pkill -f cloud-hypervisor` may hang; use `kill -9 <pid>` directly
- Need to kill old VM before starting new one (port conflict + disk lock)
