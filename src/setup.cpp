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
