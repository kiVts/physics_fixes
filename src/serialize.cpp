
#include "game.h"
#include "hooks.h"
#include "log.h"
#include "sdk.h"

#include "libdeflate.h"
#include "phys_pool.h"
#include "world.h"

#include <windows.h>
#include <algorithm>
#include <atomic>
#include <cstdint>
#include <cstring>
#include <vector>

void* rp_serialize_heap_enter(); // render_mt.cpp
void rp_serialize_heap_leave(void* prev);
bool rp_serialize_heap_ready();
void rp_serialize_pool_stats(uint32_t* slabs, uint64_t* blocks);

namespace {

constexpr uintptr_t kWriteXml = 0x14090da80;    // (writer, ctx, doc, element name, object): object -> XML element
constexpr uintptr_t kWriteState = 0x1404c5030;  // (vehicle, ctx, writer, ?): vehicle state -> binary
constexpr uintptr_t kVehicleRequest = 0x140784820; // host: client request 0x1b, one vehicle's data for one player
constexpr uintptr_t kWorldSave = 0x1407749e0;   // world save: the scene document, on the calling thread
constexpr uintptr_t kSpawnTick = 0x140782110;   // host, every tick: workbench spawns (writes each new vehicle's XML to disk)
constexpr uintptr_t kCompress = 0x14017da20;    // (dst, u32* dst_len in/out, src, src_len) -> 0 ok / miniz error
constexpr uintptr_t kRequests = 0x140851630;    // host: one player's bundle of requests (game, ?, session, bundle, ?)
constexpr uintptr_t kDocText = 0x1401f2e70;     // (document holder, Buf* out): the document as text
constexpr uintptr_t kPack = 0x14021a9b0;        // (const Buf* in, Buf* out): u32 length + zlib stream
constexpr uintptr_t kSizedDelete = 0x14095881c, kAlignedFree = 0x1409b2d50;

using Fn6 = uint64_t(__fastcall*)(uint64_t, uint64_t, uint64_t, uint64_t, uint64_t, uint64_t);
using Fn2 = uint64_t(__fastcall*)(uint64_t, uint64_t);
using Compress_t = int(__fastcall*)(uint8_t* dst, uint32_t* dst_len, const uint8_t* src, uint32_t src_len);
using New_t = void*(__fastcall*)(size_t);
using DocCtor_t = void*(__fastcall*)(void* mem, uint32_t flag);
using Reset_t = void(__fastcall*)(void** slot, void* doc); // the holder's document: deletes the old one
using SizedDelete_t = void(__fastcall*)(void* p, size_t n);
using Free_t = void(__cdecl*)(void* p);

Fn6 g_xml_orig = nullptr, g_state_orig = nullptr, g_request_orig = nullptr, g_save_orig = nullptr, g_spawn_orig = nullptr;
Fn6 g_requests_orig = nullptr;
Fn2 g_text_orig = nullptr, g_pack_orig = nullptr;
Compress_t g_compress_orig = nullptr;

bool g_private_heap = true, g_fast_compress = true;
int g_level = 6;
std::atomic<int> g_verify_left{64};
std::atomic<bool> g_fast_ok{true};
double g_log_ms = 20;
double g_tick_ms = 0;

double now_ms()
{
    LARGE_INTEGER c;
    QueryPerformanceCounter(&c);
    return static_cast<double>(c.QuadPart) * g_tick_ms;
}

// What one request / save spent where (this thread; nested writer calls count once).
struct Episode {
    double xml = 0, state = 0, compress = 0, verify = 0; // verify: the first calls' self-check, not in compress
    uint64_t in = 0, out = 0;
    int compresses = 0;
};
thread_local Episode tl_ep;
thread_local int tl_outer = 0, tl_xml = 0, tl_state = 0;

uint64_t run_writer(Fn6 orig, int& depth, double Episode::*slot, uint64_t a, uint64_t b, uint64_t c, uint64_t d,
                    uint64_t e, uint64_t f)
{
    void* heap = g_private_heap ? rp_serialize_heap_enter() : nullptr;
    double t0 = depth++ == 0 ? now_ms() : 0;
    uint64_t r = orig(a, b, c, d, e, f);
    if (--depth == 0) tl_ep.*slot += now_ms() - t0;
    if (g_private_heap) rp_serialize_heap_leave(heap);
    return r;
}

uint64_t __fastcall state_detour(uint64_t a, uint64_t b, uint64_t c, uint64_t d, uint64_t e, uint64_t f)
{
    return run_writer(g_state_orig, tl_state, &Episode::state, a, b, c, d, e, f);
}

int __fastcall compress_detour(uint8_t* dst, uint32_t* dst_len, const uint8_t* src, uint32_t src_len)
{
    double t0 = now_ms(), verify0 = tl_ep.verify;
    int r = -1;
    if (g_fast_compress && g_fast_ok.load(std::memory_order_relaxed) && dst && dst_len && (src || !src_len)) {
        thread_local libdeflate_compressor* c = nullptr; // one per thread (update thread, save thread, pool threads)
        if (!c) c = libdeflate_alloc_compressor(g_level);
        size_t n = c ? libdeflate_zlib_compress(c, src, src_len, dst, *dst_len) : 0;
        if (n) {
            if (g_verify_left.load(std::memory_order_relaxed) > 0) {
                g_verify_left.fetch_sub(1);
                double v0 = now_ms();
                std::vector<uint8_t> back(src_len);
                size_t got = 0;
                libdeflate_decompressor* dz = libdeflate_alloc_decompressor();
                bool same = dz && libdeflate_zlib_decompress(dz, dst, n, back.data(), back.size(), &got) == LIBDEFLATE_SUCCESS &&
                            got == src_len && memcmp(back.data(), src, src_len) == 0;
                if (dz) libdeflate_free_decompressor(dz);
                tl_ep.verify += now_ms() - v0;
                if (!same) {
                    g_fast_ok.store(false);
                    swlog("serialize: fast compression check FAILED (%u bytes); back to the game's compressor", src_len);
                    n = 0;
                }
            }
            if (n) {
                *dst_len = static_cast<uint32_t>(n);
                r = 0;
            }
        }
    }
    if (r != 0) r = g_compress_orig(dst, dst_len, src, src_len); // off, didn't fit, or failed the check
    tl_ep.compress += now_ms() - t0 - (tl_ep.verify - verify0);
    tl_ep.in += src_len;
    if (r == 0 && dst_len) tl_ep.out += *dst_len;
    tl_ep.compresses++;
    return r;
}

// ---- vehicle requests in parallel (parallel_requests, see the top) ----

struct Buf { // the game's byte buffer: _aligned_malloc'd, owned
    uint8_t* ptr;
    uint32_t size, pad;
};
struct Holder {
    uint8_t* buf;
    uint32_t n, pad0;
    void* p10; // a 0x18-byte object
    void* doc;
    uint32_t x;
    uint8_t flag, pad1[3];
};
struct WriteCtx {
    const uint8_t *a, *b;
    uint32_t five, pad;
};
// Read from the live handler at install.
struct Layout {
    size_t ring1 = 0, ring2 = 0, ctx_a = 0, ctx_b = 0;
    const void* name = nullptr;
    New_t fnew = nullptr;
    DocCtor_t ctor = nullptr;
    Reset_t reset = nullptr;
} g_lay;
SizedDelete_t g_sized_delete = nullptr;
Free_t g_free = nullptr;

constexpr uint32_t kVehicleData = 0x1b;
constexpr size_t kBundleRing = 0x18, kRequestKind = 8, kRequestVehicle = 0x10;
constexpr size_t kMaxBatch = 64;

struct Prep {
    const uint8_t* req = nullptr;
    uint32_t id = 0;
    int state = 0;      // 0 to do, 1 ready, 2 no such vehicle (the game's call returns early too), -1 failed, 3 done
    bool check = false; // the game's own call runs; its text is compared with this one
    Buf text{}, packed{};
    double ms = 0;
};
struct Batch {
    std::vector<Prep> preps;
    const uint8_t* world = nullptr;
    std::atomic<int> next{0};
};

bool g_par = true;
std::atomic<bool> g_par_ok{true};
std::atomic<int> g_check_left{8};
uint64_t g_check_count = 0, g_used = 0, g_checks_ok = 0;
thread_local const uint8_t* tl_bundle = nullptr;
thread_local Batch* tl_batch = nullptr;
thread_local Prep* tl_use = nullptr;   // this call takes the prepared buffer
thread_local Prep* tl_check = nullptr; // this call runs as before and is compared
thread_local int tl_phase = 0;         // tl_use: 0 before the XML step, 1 skipped it, 2 skipped the text, 3 handed over
thread_local bool tl_checked = false;

void free_buf(Buf& b)
{
    if (b.ptr) g_free(b.ptr);
    b = Buf{};
}

const uint8_t* ring_find(const uint8_t* w, size_t ring, uint32_t id)
{
    const uint8_t* const* data = *reinterpret_cast<const uint8_t* const* const*>(w + ring);
    uint32_t cap = *reinterpret_cast<const uint32_t*>(w + ring + 8), head = *reinterpret_cast<const uint32_t*>(w + ring + 12),
             count = *reinterpret_cast<const uint32_t*>(w + ring + 16);
    if (!data || !cap) return nullptr;
    for (uint32_t i = 0; i < count; i++) {
        const uint8_t* e = data[(head + i) % cap];
        if (e && *reinterpret_cast<const uint32_t*>(e + 4) == id) return e;
    }
    return nullptr;
}

int prep_one(Prep& p, const uint8_t* w)
{
    const uint8_t* e = ring_find(w, g_lay.ring1, p.id);
    if (!e || !ring_find(w, g_lay.ring2, p.id)) return 2;
    Holder h{};
    h.flag = 1;
    void* doc = g_lay.ctor(g_lay.fnew(0x4a0), h.flag);
    g_lay.reset(&h.doc, doc);
    WriteCtx c{w + g_lay.ctx_a, w + g_lay.ctx_b, 5, 0};
    run_writer(g_xml_orig, tl_xml, &Episode::xml, reinterpret_cast<uint64_t>(&h), reinterpret_cast<uint64_t>(&c),
               h.x == 0 ? reinterpret_cast<uint64_t>(h.doc) : 0, reinterpret_cast<uint64_t>(g_lay.name),
               *reinterpret_cast<const uint64_t*>(e + 8), 0);
    g_text_orig(reinterpret_cast<uint64_t>(&h), reinterpret_cast<uint64_t>(&p.text));
    g_pack_orig(reinterpret_cast<uint64_t>(&p.text), reinterpret_cast<uint64_t>(&p.packed));
    g_lay.reset(&h.doc, nullptr);
    if (h.p10) g_sized_delete(h.p10, 0x18);
    if (h.n && h.buf) g_free(h.buf);
    if (!p.check) free_buf(p.text);
    return 1;
}

int prep_safe(Prep& p, const uint8_t* w)
{
    __try {
        return prep_one(p, w);
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        return -1;
    }
}

void prep_worker(int, void* ctx)
{
    Batch* b = static_cast<Batch*>(ctx);
    for (;;) {
        int i = b->next.fetch_add(1);
        if (i >= static_cast<int>(b->preps.size())) return;
        Prep& p = b->preps[i];
        double t0 = now_ms();
        p.state = prep_safe(p, b->world);
        p.ms = now_ms() - t0;
    }
}

// The bundle's requests from `req` on, while they are vehicle requests (update thread): ids and pointers.
int collect_safe(const uint8_t* bundle, const uint8_t* req, const uint8_t** reqs, uint32_t* ids, int max)
{
    __try {
        const uint8_t* ring = bundle + kBundleRing;
        const uint8_t* const* data = *reinterpret_cast<const uint8_t* const* const*>(ring);
        uint32_t cap = *reinterpret_cast<const uint32_t*>(ring + 8), head = *reinterpret_cast<const uint32_t*>(ring + 12),
                 count = *reinterpret_cast<const uint32_t*>(ring + 16);
        if (!data || !cap || count > 100000) return 0;
        uint32_t i = 0;
        while (i < count && data[(head + i) % cap] != req) i++;
        int n = 0;
        for (; i < count && n < max; i++) {
            const uint8_t* r = data[(head + i) % cap];
            if (!r || *reinterpret_cast<const uint32_t*>(r + kRequestKind) != kVehicleData) break;
            reqs[n] = r;
            ids[n++] = *reinterpret_cast<const uint32_t*>(r + kRequestVehicle);
        }
        return n;
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        return 0;
    }
}

Prep* find_prep(const uint8_t* req)
{
    if (!tl_batch) return nullptr;
    for (Prep& p : tl_batch->preps)
        if (p.req == req) return &p;
    return nullptr;
}

void drop_batch()
{
    if (!tl_batch) return;
    for (Prep& p : tl_batch->preps) {
        free_buf(p.text);
        free_buf(p.packed);
    }
    delete tl_batch;
    tl_batch = nullptr;
}

void prepare_batch(const uint8_t* w38, const uint8_t* req)
{
    drop_batch(); // an earlier run in this bundle that ended (a request of another kind came between)
    const uint8_t* reqs[kMaxBatch];
    uint32_t ids[kMaxBatch];
    int n = collect_safe(tl_bundle, req, reqs, ids, static_cast<int>(kMaxBatch));
    if (n < 2 || reqs[0] != req) return; // one request alone: nothing to overlap
    // Bodies still moving and the pool in use while a step runs: wait for it (it's the same step the next simulate
    // waits for at zone 0x11).
    double w0 = now_ms();
    while (physics_step_running() && now_ms() - w0 < 25) SwitchToThread();
    double waited = now_ms() - w0;
    if (physics_step_running()) {
        swlog("serialize: %d vehicle requests one at a time (the physics step still ran after %.1f ms)", n, waited);
        return;
    }
    auto* b = new Batch();
    b->world = *reinterpret_cast<const uint8_t* const*>(w38);
    b->preps.resize(n);
    for (int i = 0; i < n; i++) {
        Prep& p = b->preps[i];
        p.req = reqs[i];
        p.id = ids[i];
        p.check = g_check_left.load(std::memory_order_relaxed) > 0 ? g_check_left.fetch_sub(1) > 0 : ++g_check_count % 16 == 0;
    }
    int threads = std::min(phys_pool_threads(), n);
    double t0 = now_ms();
    phys_pool_run(threads - 1, &prep_worker, b);
    double wall = now_ms() - t0, work = 0;
    int ready = 0, missing = 0, failed = 0;
    for (const Prep& p : b->preps) {
        work += p.ms;
        ready += p.state == 1;
        missing += p.state == 2;
        failed += p.state < 0;
    }
    swlog("serialize: %d vehicle requests from one player written in parallel: %.1f ms on up to %d threads (%.1f ms of "
          "work; waited %.1f ms for the physics step); %d ready, %d without a vehicle, %d failed",
          n, wall, threads, work, waited, ready, missing, failed);
    if (failed) {
        g_par_ok.store(false);
        swlog("serialize: writing a vehicle on a pool thread failed; vehicle requests one at a time again");
    }
    tl_batch = b;
}

// After the game's call for a prepared request.
void finish(Prep* p)
{
    if (tl_use == p && tl_phase == 3) g_used++;
    else if (tl_use == p && tl_phase != 0) { // skipped a step but never handed the buffer over
        g_par_ok.store(false);
        swlog("serialize: a prepared vehicle request stopped after step %d; vehicle requests one at a time again", tl_phase);
    }
    if (tl_check == p && !tl_checked) swlog("serialize: a checked vehicle request never reached the pack step");
    free_buf(p->text);
    free_buf(p->packed);
    p->state = 3;
    tl_use = tl_check = nullptr;
    tl_checked = false;
    tl_phase = 0;
}

uint64_t __fastcall requests_detour(uint64_t a, uint64_t b, uint64_t c, uint64_t d, uint64_t e, uint64_t f)
{
    const uint8_t* prev = tl_bundle;
    tl_bundle = reinterpret_cast<const uint8_t*>(d);
    uint64_t r = g_requests_orig(a, b, c, d, e, f);
    tl_bundle = prev;
    drop_batch();
    return r;
}

uint64_t __fastcall text_detour(uint64_t holder, uint64_t out)
{
    if (tl_use && tl_phase == 1) {
        tl_phase = 2;
        return 0;
    }
    return g_text_orig(holder, out);
}

uint64_t __fastcall pack_detour(uint64_t in, uint64_t out)
{
    if (Prep* p = tl_use; p && tl_phase == 2) {
        tl_phase = 3;
        Buf* o = reinterpret_cast<Buf*>(out);
        free_buf(*o);
        *o = p->packed;
        p->packed = Buf{};
        return 0;
    }
    if (Prep* p = tl_check; p && !tl_checked) {
        tl_checked = true;
        const Buf* i = reinterpret_cast<const Buf*>(in);
        bool same = i->size == p->text.size && (!i->size || memcmp(i->ptr, p->text.ptr, i->size) == 0);
        if (same) {
            if (++g_checks_ok <= 8 || g_checks_ok % 64 == 0)
                swlog("serialize: vehicle written in parallel matches the game's own (%u bytes; %llu checked, %llu used)", i->size,
                      static_cast<unsigned long long>(g_checks_ok), static_cast<unsigned long long>(g_used));
        } else {
            g_par_ok.store(false);
            swlog("serialize: vehicle written in parallel DIFFERS from the game's own (%u vs %u bytes); vehicle requests one at "
                  "a time again",
                  p->text.size, i->size);
        }
    }
    return g_pack_orig(in, out);
}

constexpr uint16_t W = 0x100; // any byte
const uint16_t kLookups[] = {0x4C, 0x8B, 0x11, 0x45, 0x8B, 0x5E, 0x10, 0x45, 0x8B, 0xC4, 0x45, 0x8B, 0x8A, W,    W,    W,    W,
                             0x0F, 0x1F, 0x40, 0x00, 0x45, 0x3B, 0xC1, 0x74, 0x2D, 0x41, 0x8B, 0x82, W,    W,    W,    W,
                             0x41, 0x03, 0xC0, 0x33, 0xD2, 0x41, 0xF7, 0xB2, W,    W,    W,    W,    0x49, 0x8B, 0x82, W,
                             W,    W,    W,    0x48, 0x8B, 0x1C, 0xD0, 0x48, 0x89, 0x5D, 0x48, 0x44, 0x39, 0x5B, 0x04, 0x74,
                             0x0C, 0x41, 0xFF, 0xC0, 0xEB, 0xCE, 0x49, 0x8B, 0xDC, 0x48, 0x89, 0x5D, 0x48, 0x45, 0x8B, 0xC4,
                             0x45, 0x8B, 0x4A, W,    0x45, 0x3B, 0xC1, 0x74, 0x24, 0x41, 0x8B, 0x42, W,    0x41, 0x03, 0xC0,
                             0x33, 0xD2, 0x41, 0xF7, 0x72, W,    0x49, 0x8B, 0x42, W,    0x48, 0x8B, 0x04, 0xD0, 0x48, 0x89,
                             0x45, 0x50, 0x44, 0x39, 0x58, 0x04};
const uint16_t kWrite[] = {0x4C, 0x89, 0x65, 0x20, 0x44, 0x89, 0x65, 0x28, 0x4C, 0x89, 0x65, 0x30, 0x4C, 0x89, 0x65, 0x38, 0x44, 0x89,
                           0x65, 0x40, 0xC6, 0x45, 0x44, 0x01, 0xB9, 0xA0, 0x04, 0x00, 0x00, 0xE8, W,    W,    W,    W,    0x48, 0x89,
                           0x85, 0x90, 0x00, 0x00, 0x00, 0x0F, 0xB6, 0x55, 0x44, 0x48, 0x8B, 0xC8, 0xE8, W,    W,    W,    W,    0x90,
                           0x48, 0x8B, 0xD0, 0x48, 0x8D, 0x4D, 0x38, 0xE8, W,    W,    W,    W,    0x90, 0x48, 0x8B, 0x07, 0x48, 0x8D,
                           0x88, W,    W,    W,    W,    0x48, 0x05, W,    W,    W,    W,    0x48, 0x89, 0x45, 0x78, 0x48, 0x89, 0x8D,
                           0x80, 0x00, 0x00, 0x00, 0xC7, 0x85, 0x88, 0x00, 0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x48, 0x8B, 0x43, 0x08,
                           0x4D, 0x8B, 0xC4, 0x44, 0x39, 0x65, 0x40, 0x4C, 0x0F, 0x44, 0x45, 0x38, 0x48, 0x89, 0x44, 0x24, 0x20, 0x4C,
                           0x8D, 0x0D, W,    W,    W,    W,    0x48, 0x8D, 0x55, 0x78, 0x48, 0x8D, 0x4D, 0x20, 0xE8, W,    W,    W,
                           W,    0x4C, 0x89, 0x65, 0xF8, 0x44, 0x89, 0x65, 0x00, 0x48, 0x8D, 0x55, 0xF8, 0x48, 0x8D, 0x4D, 0x20, 0xE8,
                           W,    W,    W,    W,    0x4C, 0x89, 0x65, 0x98, 0x44, 0x89, 0x65, 0xA0, 0x48, 0x8D, 0x55, 0x98, 0x48, 0x8D,
                           0x4D, 0xF8, 0xE8, W,    W,    W,    W};

template <size_t N> const uint8_t* find_unique(const uint8_t* from, const uint8_t* end, const uint16_t (&pat)[N])
{
    const uint8_t* found = nullptr;
    for (const uint8_t* p = from; p + N <= end; p++) {
        size_t i = 0;
        while (i < N && (pat[i] == W || p[i] == pat[i])) i++;
        if (i < N) continue;
        if (found) return nullptr; // not unique
        found = p;
    }
    return found;
}

// The pattern's wildcard runs in order: a 1-byte run as an unsigned displacement, a 4-byte run as a displacement, or,
// after E8 / lea r9,[rip+x], the address it points at.
template <size_t N> int wild_values(const uint8_t* at, const uint16_t (&pat)[N], uintptr_t* out, int max)
{
    int k = 0;
    for (size_t i = 0; i < N && k < max;) {
        if (pat[i] != W) {
            i++;
            continue;
        }
        size_t n = 0;
        while (i + n < N && pat[i + n] == W) n++;
        if (n == 1) {
            out[k++] = at[i];
        } else {
            int32_t d;
            memcpy(&d, at + i, 4);
            bool rel = pat[i - 1] == 0xE8 || (i >= 3 && pat[i - 3] == 0x4C && pat[i - 2] == 0x8D && pat[i - 1] == 0x0D);
            out[k++] = rel ? reinterpret_cast<uintptr_t>(at + i + 4) + d : static_cast<uint32_t>(d);
        }
        i += n;
    }
    return k;
}

// fn = the live handler; xml / text / pack = where its three calls must go.
const char* parse_layout(const uint8_t* fn, uintptr_t xml, uintptr_t text, uintptr_t pack, Layout& lay)
{
    const uint8_t* a = find_unique(fn + 0x10, fn + 0x300, kLookups);
    const uint8_t* b = find_unique(fn + 0x10, fn + 0x300, kWrite);
    if (!a || !b) return "handler code not recognised";
    uintptr_t l[8], w[9];
    // lookups: count, head, cap, data of ring 1 (disp32), then of ring 2 (disp8)
    if (wild_values(a, kLookups, l, 8) != 8 || l[0] != l[3] + 16 || l[1] != l[3] + 12 || l[2] != l[3] + 8 || l[4] != l[7] + 16 ||
        l[5] != l[7] + 12 || l[6] != l[7] + 8)
        return "vehicle lookups not as expected";
    // write: new, document constructor, holder reset, ctx_b, ctx_a, name, XML, text, pack
    if (wild_values(b, kWrite, w, 9) != 9 || w[6] != xml || w[7] != text || w[8] != pack)
        return "the handler's XML / text / pack calls not as expected";
    lay.ring1 = l[3];
    lay.ring2 = l[7];
    lay.fnew = reinterpret_cast<New_t>(w[0]);
    lay.ctor = reinterpret_cast<DocCtor_t>(w[1]);
    lay.reset = reinterpret_cast<Reset_t>(w[2]);
    lay.ctx_b = w[3];
    lay.ctx_a = w[4];
    lay.name = reinterpret_cast<const void*>(w[5]);
    return nullptr;
}

const char* read_layout()
{
    __try {
        return parse_layout(game_ptr<const uint8_t*>(kVehicleRequest), game_live(kWriteXml), game_live(kDocText), game_live(kPack), g_lay);
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        return "handler code unreadable";
    }
}

// ---- episodes ----

void report(const char* what, double total)
{
    const Episode& e = tl_ep;
    if (total < g_log_ms) return;
    uint32_t slabs = 0;
    uint64_t blocks = 0;
    rp_serialize_pool_stats(&slabs, &blocks);
    swlog("serialize: %s %.1f ms = XML %.1f + state %.1f + compress %.1f (%d x, %.1f MB -> %.2f MB, %s) + rest %.1f  [heap %s%s; pool %u slabs, %llu blocks live]",
          what, total, e.xml, e.state, e.compress, e.compresses, e.in / 1048576.0, e.out / 1048576.0,
          g_fast_compress && g_fast_ok.load() ? "libdeflate" : "game", total - e.xml - e.state - e.compress - e.verify,
          g_private_heap && rp_serialize_heap_ready() ? "private pool" : "shared", e.verify > 0 ? ", incl. compression self-check" : "",
          slabs, static_cast<unsigned long long>(blocks));
}

uint64_t run_episode(Fn6 orig, const char* what, uint64_t a, uint64_t b, uint64_t c, uint64_t d, uint64_t e, uint64_t f)
{
    if (tl_outer++ != 0) {
        uint64_t r = orig(a, b, c, d, e, f);
        tl_outer--;
        return r;
    }
    tl_ep = Episode{};
    double t0 = now_ms();
    uint64_t r = orig(a, b, c, d, e, f);
    report(what, now_ms() - t0);
    tl_outer--;
    return r;
}

uint64_t __fastcall xml_detour(uint64_t a, uint64_t b, uint64_t c, uint64_t d, uint64_t e, uint64_t f)
{
    if (tl_use && tl_phase == 0) { // written on the pool already: the game's own call skips it
        tl_phase = 1;
        return 0;
    }
    return run_writer(g_xml_orig, tl_xml, &Episode::xml, a, b, c, d, e, f);
}

// e = the request (the handler's 5th argument)
uint64_t __fastcall request_detour(uint64_t a, uint64_t b, uint64_t c, uint64_t d, uint64_t e, uint64_t f)
{
    const uint8_t* req = reinterpret_cast<const uint8_t*>(e);
    Prep* p = nullptr;
    if (g_par && g_par_ok.load(std::memory_order_relaxed) && tl_bundle && tl_outer == 0 && a && req) {
        p = find_prep(req);
        if (!p) {
            prepare_batch(reinterpret_cast<const uint8_t*>(a), req);
            p = find_prep(req);
        }
        if (p && p->state == 1) {
            if (p->check) {
                tl_check = p;
            } else {
                tl_use = p;
                tl_phase = 0;
            }
        }
    }
    uint64_t r = run_episode(g_request_orig,
                             tl_use ? "vehicle data for a player (request 0x1b, written in parallel)" : "vehicle data for a player (request 0x1b)",
                             a, b, c, d, e, f);
    if (p) finish(p);
    return r;
}

uint64_t __fastcall spawn_detour(uint64_t a, uint64_t b, uint64_t c, uint64_t d, uint64_t e, uint64_t f)
{
    return run_episode(g_spawn_orig, "vehicle spawn (workbench, FUN_140782110)", a, b, c, d, e, f);
}

uint64_t __fastcall save_detour(uint64_t a, uint64_t b, uint64_t c, uint64_t d, uint64_t e, uint64_t f)
{
    return run_episode(g_save_orig, "world save", a, b, c, d, e, f);
}

bool install()
{
    const wchar_t* ini = settings_ini();
    g_private_heap = GetPrivateProfileIntW(L"serialize", L"private_heap", 1, ini) != 0;
    g_fast_compress = GetPrivateProfileIntW(L"serialize", L"fast_compress", 1, ini) != 0;
    g_level = std::clamp(static_cast<int>(GetPrivateProfileIntW(L"serialize", L"level", 6, ini)), 1, 12);
    g_verify_left = static_cast<int>(GetPrivateProfileIntW(L"serialize", L"verify_calls", 64, ini));
    g_log_ms = GetPrivateProfileIntW(L"serialize", L"log_ms", 20, ini);
    g_par = GetPrivateProfileIntW(L"serialize", L"parallel_requests", 1, ini) != 0;
    g_check_left = static_cast<int>(GetPrivateProfileIntW(L"serialize", L"parallel_check", 8, ini));
    LARGE_INTEGER f;
    QueryPerformanceFrequency(&f);
    g_tick_ms = 1000.0 / static_cast<double>(f.QuadPart);

    bool ok = hook_game(kWriteXml, reinterpret_cast<void*>(&xml_detour), reinterpret_cast<void**>(&g_xml_orig)) &&
              hook_game(kWriteState, reinterpret_cast<void*>(&state_detour), reinterpret_cast<void**>(&g_state_orig)) &&
              hook_game(kVehicleRequest, reinterpret_cast<void*>(&request_detour), reinterpret_cast<void**>(&g_request_orig)) &&
              hook_game(kWorldSave, reinterpret_cast<void*>(&save_detour), reinterpret_cast<void**>(&g_save_orig)) &&
              hook_game(kSpawnTick, reinterpret_cast<void*>(&spawn_detour), reinterpret_cast<void**>(&g_spawn_orig)) &&
              hook_game(kCompress, reinterpret_cast<void*>(&compress_detour), reinterpret_cast<void**>(&g_compress_orig));
    if (!ok) return false;
    swlog("serialize: private heap %s, compression %s (level %d, first %d checked), logging calls >= %.0f ms",
          g_private_heap ? "on" : "off", g_fast_compress ? "libdeflate" : "game (miniz)", g_level, g_verify_left.load(), g_log_ms);
    if (!g_par) {
        swlog("serialize: vehicle requests in parallel off in settings.ini");
        return true;
    }
    g_sized_delete = game_ptr<SizedDelete_t>(kSizedDelete);
    g_free = game_ptr<Free_t>(kAlignedFree);
    const char* why = read_layout();
    if (!why && !(hook_game(kRequests, reinterpret_cast<void*>(&requests_detour), reinterpret_cast<void**>(&g_requests_orig)) &&
                  hook_game(kDocText, reinterpret_cast<void*>(&text_detour), reinterpret_cast<void**>(&g_text_orig)) &&
                  hook_game(kPack, reinterpret_cast<void*>(&pack_detour), reinterpret_cast<void**>(&g_pack_orig))))
        why = "hooks failed";
    g_par = !why;
    if (why) swlog("serialize: vehicle requests in parallel off (%s)", why);
    else
        swlog("serialize: vehicle requests in parallel on (a player's run of vehicle requests written on up to %d threads at "
              "once; rings W+%#zx / W+%#zx, context W+%#zx / W+%#zx; first %d checked against the game's own, then every 16th)",
              phys_pool_threads(), g_lay.ring1, g_lay.ring2, g_lay.ctx_a, g_lay.ctx_b, g_check_left.load());
    return true;
}

HookRegistrar reg{{"vehicle / world serialization (private heap, libdeflate, parallel requests, timing)", true, &install}};

} // namespace
