// physics_fixes: the builds its fixed addresses and struct offsets were checked against.
//
// 0x6ae218ec (Steam update 2026-10-28), checked with tools/port_*.py: render prep, the vehicle loop and Bullet are
// laid out as before; what moved is handled in place with per_build(): the game object's vehicle table
// (radar_msg.cpp), the vehicle loop's world sub-objects outside the shifted block (vehicle_mt.cpp).
#include "game.h"
#include "port_table.h"
#include "sdk.h"

namespace {
bool setup()
{
    mod_ported_to(kPortToBuild);
    return true;
}
ModSetup reg{&setup};
} // namespace
