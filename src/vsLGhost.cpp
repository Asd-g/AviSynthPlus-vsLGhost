#include <cstdlib>
#include <algorithm>
#include <cstring>

#include "vsLGhost.h"

template<typename pixel_t>
void vsLGhost::filter_c(PVideoFrame& src, PVideoFrame& dst, const vsLGhost* const __restrict, IScriptEnvironment* env) noexcept
{
    using var_t = std::conditional_t<std::is_integral_v<pixel_t>, int, float>;

    var_t* buffer = reinterpret_cast<var_t*>(vsLGhost::buffer);

    int planes_y[4] = { PLANAR_Y, PLANAR_U, PLANAR_V, PLANAR_A };
    int planes_r[4] = { PLANAR_G, PLANAR_B, PLANAR_R, PLANAR_A };
    const int* current_planes = (vi.IsYUV() || vi.IsYUVA()) ? planes_y : planes_r;
    const int planecount = std::min(vi.NumComponents(), 3);
    for (int i = 0; i < planecount; i++)
    {
        const int plane = current_planes[i];
        const int height = src->GetHeight(plane);

        if (process[i] == 3)
        {
            const int width = src->GetRowSize(plane) / sizeof(pixel_t);            
            const pixel_t* srcp = reinterpret_cast<const pixel_t*>(src->GetReadPtr(plane));
            pixel_t* __restrict dstp = reinterpret_cast<pixel_t*>(dst->GetWritePtr(plane));

            for (int y = 0; y < height; y++)
            {
                memset(buffer, 0, width * sizeof(var_t));

                for (auto&& it : options[i][0])
                    for (int x = it.startX; x < it.endX; x++)
                        buffer[x] += (srcp[x - it.shift + 1] - srcp[x - it.shift]) * it.intensity;

                for (auto&& it : options[i][1])
                    for (int x = it.startX; x < it.endX; x++)
                        buffer[x] += srcp[x - it.shift] * it.intensity;

                for (auto&& it : options[i][2])
                    for (int x = it.startX; x < it.endX; x++)
                        if (const var_t tempEdge = srcp[x - it.shift + 1] - srcp[x - it.shift]; tempEdge > 0)
                            buffer[x] += tempEdge * it.intensity;

                for (auto&& it : options[i][3])
                    for (int x = it.startX; x < it.endX; x++)
                        if (const var_t tempEdge = srcp[x - it.shift + 1] - srcp[x - it.shift]; tempEdge < 0)
                            buffer[x] += tempEdge * it.intensity;

                for (int x = 0; x < width; x++)
                {
                    if constexpr (std::is_integral_v<pixel_t>)
                        dstp[x] = std::clamp(srcp[x] + (buffer[x] >> 7), 0, peak);
                    else
                        dstp[x] = srcp[x] + (buffer[x] * (1.0f / 128.0f));
                }

                srcp += src->GetPitch(plane) / sizeof(pixel_t);
                dstp += dst->GetPitch(plane) / sizeof(pixel_t);
            }
        }
        else if (process[i] == 2)
            env->BitBlt(dst->GetWritePtr(plane), dst->GetPitch(plane), src->GetReadPtr(plane), src->GetPitch(plane), src->GetRowSize(plane), height);
    }
}

vsLGhost::vsLGhost(PClip _child, const std::vector<int>& mode, const std::vector<int>& shift, const std::vector<int>& intensity, int y, int u, int v, int opt, IScriptEnvironment* env)
    : GenericVideoFilter(_child), mode_(mode), shift_(shift), intensity_(intensity), opt_(opt)
{
    if (!vi.IsPlanar())
        env->ThrowError("vsLGhost: the clip is not in planar format.");

    const int num_mode = mode_.size();
    const int num_shift = shift_.size();
    const int num_intensity = intensity_.size();

    if (num_mode != num_shift || num_mode != num_intensity)
        env->ThrowError("vsLGhost: the number of the elements in mode, shift and intensity must be equal.");

    int planes[3] = { y, u, v };
    const int planecount = std::min(vi.NumComponents(), 3);
    for (int i = 0; i < planecount; i++)
    {
        if (vi.IsRGB())
            process[i] = 3;
        else
        {
            switch (planes[i])
            {
                case 3: process[i] = 3; break;
                case 2: process[i] = 2; break;
                default: process[i] = 1; break;
            }
        }
    }

    for (int i = 0; i < num_mode; ++i)
    {
        if (mode_[i] < 1 || mode_[i] > 4)
            env->ThrowError("vsLGhost: mode must be 1, 2, 3, or 4.");

        if (intensity_[i] == 0 || intensity_[i] < -128 || intensity_[i] > 127)
            env->ThrowError("vsLGhost: intensity must not be 0 and must be between -128 and 127 (inclusive).");
    
        for (int plane = 0; plane < planecount; plane++)
        {
            if (process[plane] == 3)
            {
                int width = vi.width >> (plane && !vi.IsRGB() ? vi.GetPlaneWidthSubsampling(PLANAR_U) : 0);

                if (std::abs(shift_[i]) >= width)
                    env->ThrowError("vsLGhost: abs(shift) must be less than plane's width.");

                if (mode_[i] != 2)
                {
                    if (shift_[i] == 0)
                        env->ThrowError("vsLGhost: shift must not be 0 for mode 1, 3, 4.");

                    if (shift_[i] < 0)
                    {
                        shift_[i]++;
                        width--;
                    }
                }

                options[plane][mode_[i] - 1].emplace_back(OptionData{ shift_[i], intensity_[i], std::max(shift_[i], 0), std::min(width + shift_[i], width) });
            }
        }
    }

    if (opt_ < -1 || opt_ > 3)
        env->ThrowError("vsLGhost: opt must be between -1..3.");
    if (!(env->GetCPUFlags() & CPUF_AVX512F) && opt_ == 3)
        env->ThrowError("vsLGhost: opt=3 requires AVX512F.");
    if (!(env->GetCPUFlags() & CPUF_AVX2) && opt_ == 2)
        env->ThrowError("vsLGhost: opt=2 requires AVX2.");
    if (!(env->GetCPUFlags() & CPUF_SSE2) && opt_ == 1)
        env->ThrowError("vsLGhost: opt=1 requires SSE2.");

    peak = 0;
    if (vi.ComponentSize() != 4)
        peak = (1 << vi.BitsPerComponent()) - 1;

    buffer = _aligned_malloc(static_cast<int64_t>(vi.width) * vi.height * vi.ComponentSize() * sizeof(int), 64);
    if (!buffer)
        env->ThrowError("vsLGhost: malloc failure (buffer).");

    has_at_least_v8 = true;
    try { env->CheckVersion(8); }
    catch (const AvisynthError&) { has_at_least_v8 = false; }
}

vsLGhost::~vsLGhost()
{
    _aligned_free(buffer);
}

PVideoFrame __stdcall vsLGhost::GetFrame(int n, IScriptEnvironment* env)
{
    PVideoFrame src = child->GetFrame(n, env);
    PVideoFrame dst = has_at_least_v8 ? env->NewVideoFrameP(vi, &src) : env->NewVideoFrame(vi);

    if ((!!(env->GetCPUFlags() & CPUF_AVX512F) && opt_ < 0) || opt_ == 3)
    {
        switch (vi.ComponentSize())
        {
            case 1: filter_avx512<uint8_t>(src, dst, 0, env); break;
            case 2: filter_avx512<uint16_t>(src, dst, 0, env); break;
            default: filter_avx512<float>(src, dst, 0, env); break;
        }
    }
    else if ((!!(env->GetCPUFlags() & CPUF_AVX2) && opt_ < 0) || opt_ == 2)
    {
        switch (vi.ComponentSize())
        {
            case 1: filter_avx2<uint8_t>(src, dst, 0, env); break;
            case 2: filter_avx2<uint16_t>(src, dst, 0, env); break;
            default: filter_avx2<float>(src, dst, 0, env); break;
        }
    }
    else if ((!!(env->GetCPUFlags() & CPUF_SSE2) && opt_ < 0) || opt_ == 1)
    {
        switch (vi.ComponentSize())
        {
            case 1: filter_sse2<uint8_t>(src, dst, 0, env); break;
            case 2: filter_sse2<uint16_t>(src, dst, 0, env); break;
            default: filter_sse2<float>(src, dst, 0, env); break;
        }
    }
    else
    {
        switch (vi.ComponentSize())
        {
            case 1: filter_c<uint8_t>(src, dst, 0, env); break;
            case 2: filter_c<uint16_t>(src, dst, 0, env); break;
            default: filter_c<float>(src, dst, 0, env); break;
        }
    }

    return dst;
}

AVSValue __cdecl Create_vsLGhost(AVSValue args, void* user_data, IScriptEnvironment* env)
{
    std::vector<int> mode;

    for (int i = 0; i < args[1].ArraySize(); ++i)
    {
        mode.push_back(args[1][i].AsInt());
    }

    std::vector<int> shift;
    for (int i = 0; i < args[2].ArraySize(); ++i)
    {
        shift.push_back(args[2][i].AsInt());
    }

    std::vector<int> intensity;
    for (int i = 0; i < args[3].ArraySize(); ++i)
    {
        intensity.push_back(args[3][i].AsInt());
    }


    return new vsLGhost(
        args[0].AsClip(),
        mode,
        shift,
        intensity,
        args[4].AsInt(3),
        args[5].AsInt(2),
        args[6].AsInt(2),
        args[7].AsInt(-1),
        env);
}

const AVS_Linkage* AVS_linkage;

extern "C" __declspec(dllexport)
const char* __stdcall AvisynthPluginInit3(IScriptEnvironment * env, const AVS_Linkage* const vectors)
{
    AVS_linkage = vectors;

    env->AddFunction("vsLGhost", "c[mode]i+[shift]i+[intensity]i+[y]i[u]i[v]i[opt]i", Create_vsLGhost, 0);

    return "vsLGhost";
}
