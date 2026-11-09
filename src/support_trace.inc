// Temporary, read-only contact-to-solver trace. Called on the physics thread except
// setup/finish, whose units have already passed the disjoint-body check.
struct SolverPointWatch { const uint8_t* point; double ny; bool ground; };
struct SolverBodyWatch {
    const uint8_t* body = nullptr;
    SolverPointWatch points[64]{};
    int np = 0, grounds = 0, body_entries = 0, units = 0, rows = 0;
    int tag = -1, activation = -1, ground_flags = 0, ground_tag = -1;
    int solver_slot = -1, map_matches = 0;
    double solver_inv_mass_y = 0;
    double y = 0, vy = 0, depth = 0, ground_impulse = 0, other_impulse = 0;
};
SolverBodyWatch g_solver_watch[16];
const uint8_t* g_solver_last_body[16]{};
double g_solver_rest_y[16]{};
int g_solver_watch_n = 0, g_solver_trace_lines = 0;
bool g_solver_native = false, g_solver_ab = false;
unsigned g_solver_trace_step = 0;

void solver_trace_begin(const uint8_t* world)
{
    g_solver_watch_n = 0;
    if (!g_support_on || !g_is_host || *g_is_host) return;
    ++g_solver_trace_step;
    int no = rd<int>(world, kWorldObjCount);
    auto objects = rd<const uint8_t* const*>(world, kWorldObjData);
    auto dispatcher = rd<const uint8_t*>(world, 0x28);
    int nm = dispatcher ? rd<int>(dispatcher, 0x14) : 0;
    auto manifolds = dispatcher ? rd<const uint8_t* const*>(dispatcher, 0x20) : nullptr;
    if (no < 0 || no > 1000000 || nm < 0 || nm > 1000000 || (no && !objects) || (nm && !manifolds)) return;
    ULONGLONG now = GetTickCount64();
    for (int slot = 0; slot < 16; ++slot) {
        auto b = g_support[slot].body;
        if (!b || now > g_support[slot].until) continue;
        bool alive = false;
        for (int i = 0; i < no; ++i) if (objects[i] == b) { alive = true; break; }
        if (!alive) continue;
        auto so = rd<const uint8_t*>(b, 0x1f8);
        auto owner = so ? rd<const uint8_t*>(so, 0xc0) : nullptr;
        if (!owner || rd<int>(owner, 0) != 3 || rd<int>(owner, 0x40) != 0x17) continue;
        auto& w = g_solver_watch[g_solver_watch_n++];
        w = {}; w.body = b;
        w.y = rd<double>(b, 0x78); w.vy = rd<double>(b, 0x398);
        w.tag = rd<int>(b, 0x1a4); w.activation = rd<int>(b, 0x1b0);
        if (g_solver_last_body[slot] != b) {
            g_solver_last_body[slot] = b; g_solver_rest_y[slot] = w.y;
        }
        for (int i = 0; i < nm; ++i) {
            auto m = manifolds[i];
            if (!m) continue;
            auto a = rd<const uint8_t*>(m, 0x5d0), other = rd<const uint8_t*>(m, 0x5d8);
            bool first = a == b;
            if (!first && other != b) continue;
            if (!first) other = a;
            int of = rd<int>(other, kObjFlags);
            if ((of & 4) || (rd<int>(b, kObjFlags) & 4) || rd<int>(other, kObjInternalType) != 2) continue;
            int n = rd<int>(m, 0x5e0);
            if (n < 0 || n > 4) continue;
            for (int j = 0; j < n && w.np < 64; ++j) {
                auto point = m + 0x10 + j * 0x170;
                double ny = rd<double>(point, 0x88) * (first ? 1 : -1);
                double d = rd<double>(point, 0xa0);
                w.points[w.np++] = {point, ny, (of & 1) != 0};
                if ((of & 1) && ny > .5 && d < .02) {
                    w.grounds++; w.depth = (std::min)(w.depth, d);
                    w.ground_flags = of; w.ground_tag = rd<int>(other, 0x1a4);
                }
            }
        }
        if (w.grounds && std::fabs(w.vy) < .05) g_solver_rest_y[slot] = (std::max)(g_solver_rest_y[slot], w.y);
        if (g_solver_ab && !g_solver_native && w.y < g_solver_rest_y[slot] - .15 && w.vy < -.5) {
            g_solver_native = true;
            swlog("pallet solver A/B: pallet %p sank %.3f m with parallel solver; untouched native solver for the rest of this session (try a fresh pallet)",
                  b, g_solver_rest_y[slot] - w.y);
        }
    }
}

void solver_trace_setup(void* solver, void** bodies, int nbodies, void** manifolds, int nmanifolds)
{
    if (!g_solver_watch_n) return;
    for (int k = 0; k < g_solver_watch_n; ++k) {
        auto& w = g_solver_watch[k];
        int entries = 0;
        for (int j = 0; j < nbodies; ++j) entries += bodies[j] == w.body;
        bool referenced = entries != 0;
        for (int j = 0; j < nmanifolds && !referenced; ++j)
            referenced = rd<const void*>(manifolds[j], 0x5d0) == w.body || rd<const void*>(manifolds[j], 0x5d8) == w.body;
        if (referenced) {
            w.units++; w.body_entries += entries;
            int id = rd<int>(w.body,0x1a8), ns = rd<int>(solver,0xc);
            auto data = rd<const uint8_t*>(solver,0x18);
            w.solver_slot = id;
            if (id >= 0 && id < ns && data) {
                auto sb = data + id * 0x1f0;
                w.map_matches = rd<const void*>(sb,0x1e0) == w.body;
                w.solver_inv_mass_y = rd<double>(sb,0x108);
            }
        }
    }
    (void)solver;
}

void solver_trace_finish(void* solver)
{
    if (!g_solver_watch_n) return;
    int nr = rd<int>(solver, 0x2c);
    auto rows = rd<const uint8_t*>(solver, 0x38);
    if (nr < 0 || nr > 1000000 || (nr && !rows)) return;
    for (int i = 0; i < nr; ++i) {
        auto row = rows + i * 0x120;
        auto point = rd<const uint8_t*>(row, 0x108);
        for (int k = 0; k < g_solver_watch_n; ++k) {
            auto& w = g_solver_watch[k];
            for (int j = 0; j < w.np; ++j) if (w.points[j].point == point) {
                w.rows++;
                double iy = rd<double>(row, 0xc8) * w.points[j].ny;
                if (w.points[j].ground) w.ground_impulse += iy;
                else w.other_impulse += iy;
            }
        }
    }
}

void solver_trace_end()
{
    for (int k = 0; k < g_solver_watch_n; ++k) {
        auto& w = g_solver_watch[k];
        if (g_solver_trace_lines >= 2400 || (std::fabs(w.vy) < .04 && g_solver_trace_step % 30)) continue;
        ++g_solver_trace_lines;
        swlog("pallet solver (%s): step %u body %p y %.4f vy %.4f -> %.4f tag %d act %d; response %d ground %d depth %.4f groundFlags 0x%x tag %d; units %d bodyEntries %d map %d matches %d invMassY %.4f rows %d impulseY ground %.4f other %.4f",
              g_solver_native ? "native" : "parallel", g_solver_trace_step, w.body, w.y, w.vy, rd<double>(w.body, 0x398),
              w.tag, w.activation, w.np, w.grounds, w.depth, w.ground_flags, w.ground_tag,
              w.units, w.body_entries, w.solver_slot, w.map_matches, w.solver_inv_mass_y, w.rows, w.ground_impulse, w.other_impulse);
    }
}
