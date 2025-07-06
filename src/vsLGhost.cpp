#include <algorithm>
#include <cstdlib>
#include <cstring>

#include "vsLGhost.h"

template<typename pixel_t>
void vsLGhost::filter_c(PVideoFrame& src, PVideoFrame& dst, IScriptEnvironment* env) const noexcept
{
    using var_t = std::conditional_t<std::is_integral_v<pixel_t>, int, float>;

    constexpr int planes_y[4] = {PLANAR_Y, PLANAR_U, PLANAR_V, PLANAR_A};
    constexpr int planes_r[4] = {PLANAR_G, PLANAR_B, PLANAR_R, PLANAR_A};
    const int* current_planes = (vi.IsYUV() || vi.IsYUVA()) ? planes_y : planes_r;

    for (int i = 0; i < planecount; i++)
    {
        const int plane = current_planes[i];
        const int height = src->GetHeight(plane);

        if (process[i] == 3)
        {
            const int width = src->GetRowSize(plane) / sizeof(pixel_t);
            const pixel_t* srcp = reinterpret_cast<const pixel_t*>(src->GetReadPtr(plane));
            pixel_t* AVS_RESTRICT dstp = reinterpret_cast<pixel_t*>(dst->GetWritePtr(plane));
            var_t* AVS_RESTRICT buffer = reinterpret_cast<var_t*>(buffer_plane[i].get());

            for (int y = 0; y < height; y++)
            {
                std::memset(buffer, 0, width * sizeof(var_t));

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
                        dstp[x] = std::clamp(static_cast<int>(srcp[x]) + (buffer[x] >> 7), 0, peak);
                    else
                        dstp[x] = srcp[x] + (buffer[x] * (1.0f / 128.0f));
                }

                srcp += src->GetPitch(plane) / sizeof(pixel_t);
                dstp += dst->GetPitch(plane) / sizeof(pixel_t);
            }
        }
        else if (process[i] == 2)
            env->BitBlt(dst->GetWritePtr(plane), dst->GetPitch(plane), src->GetReadPtr(plane), src->GetPitch(plane), src->GetRowSize(plane),
                height);
    }
}

vsLGhost::vsLGhost(PClip _child, std::vector<int> mode, std::vector<int> shift, std::vector<int> intensity, int y, int u, int v, int opt,
    IScriptEnvironment* env)
    : GenericVideoFilter(_child),
      mode_(std::move(mode)),
      shift_(std::move(shift)),
      intensity_(std::move(intensity)),
      process([&] { return vi.IsRGB() ? std::array<int, 3>{3, 3, 3} : std::array<int, 3>{y, u, v}; }()),
      peak((vi.ComponentSize() != 4) ? ((1 << vi.BitsPerComponent()) - 1) : 0),
      has_at_least_v8(env->FunctionExists("propShow")),
      planecount(std::min(vi.NumComponents(), 3)),

      plane_width([&]() {
          std::array<int, 3> widths{};
          const int width_subs = (!vi.IsRGB() && planecount > 1) ? vi.GetPlaneWidthSubsampling(PLANAR_U) : 0;

          for (int i = 0; i < planecount; ++i)
              widths[i] = i ? (vi.width >> width_subs) : vi.width;

          return widths;
      }()),

      buffer_plane([&] {
          std::array<aligned_unique_ptr<std::byte>, 3> bufs;

          for (int i = 0; i < planecount; ++i)
          {
              const size_t buffer_size = (plane_width[i] + 15) * sizeof(int);
              bufs[i] = make_unique_aligned_array<std::byte>(buffer_size, 64);

              if (!bufs[i])
                  env->ThrowError("vsLGhost: allocation failure (buffer %d).", i);
          }

          return bufs;
      }())
{
    if (!vi.IsPlanar())
        env->ThrowError("vsLGhost: the clip is not in planar format.");

    const int num_mode = mode_.size();
    const int num_shift = shift_.size();
    const int num_intensity = intensity_.size();

    if (num_mode != num_shift || num_mode != num_intensity)
        env->ThrowError("vsLGhost: the number of the elements in mode, shift and intensity must be equal.");

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
                int width = plane_width[plane];

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

                options[plane][mode_[i] - 1].emplace_back(
                    OptionData{shift_[i], intensity_[i], std::max(shift_[i], 0), std::min(width + shift_[i], width)});
            }
        }
    }

    const int cpu_instrucs = !(!!(env->GetCPUFlags() & CPUF_AVX512F) && (opt < 0 || opt == 3))
                                 ? !(!!(env->GetCPUFlags() & CPUF_AVX2) && (opt < 0 || opt == 2))
                                       ? !(!!(env->GetCPUFlags() & CPUF_SSE2) && (opt < 0 || opt == 1)) ? 0 : 1
                                       : 2
                                 : 3;

    if (opt < -1 || opt > 3)
        env->ThrowError("vsLGhost: opt must be between -1..3.");
    if (opt == 3 && cpu_instrucs != 3)
        env->ThrowError("vsLGhost: opt=3 requires AVX512F.");
    if (opt == 2 && cpu_instrucs != 2)
        env->ThrowError("vsLGhost: opt=2 requires AVX2.");
    if (opt == 1 && cpu_instrucs != 1)
        env->ThrowError("vsLGhost: opt=1 requires SSE2.");

    switch (cpu_instrucs)
    {
    case 3:
        switch (vi.ComponentSize())
        {
        case 1:
            filter_ptr = &vsLGhost::filter_avx512<uint8_t>;
            break;
        case 2:
            filter_ptr = &vsLGhost::filter_avx512<uint16_t>;
            break;
        default:
            filter_ptr = &vsLGhost::filter_avx512<float>;
            break;
        }
        break;
    case 2:
        switch (vi.ComponentSize())
        {
        case 1:
            filter_ptr = &vsLGhost::filter_avx2<uint8_t>;
            break;
        case 2:
            filter_ptr = &vsLGhost::filter_avx2<uint16_t>;
            break;
        default:
            filter_ptr = &vsLGhost::filter_avx2<float>;
            break;
        }
        break;
    case 1:
        switch (vi.ComponentSize())
        {
        case 1:
            filter_ptr = &vsLGhost::filter_sse2<uint8_t>;
            break;
        case 2:
            filter_ptr = &vsLGhost::filter_sse2<uint16_t>;
            break;
        default:
            filter_ptr = &vsLGhost::filter_sse2<float>;
            break;
        }
        break;
    case 0:
        switch (vi.ComponentSize())
        {
        case 1:
            filter_ptr = &vsLGhost::filter_c<uint8_t>;
            break;
        case 2:
            filter_ptr = &vsLGhost::filter_c<uint16_t>;
            break;
        default:
            filter_ptr = &vsLGhost::filter_c<float>;
            break;
        }
        break;
    default:
        break;
    }
}

PVideoFrame __stdcall vsLGhost::GetFrame(int n, IScriptEnvironment* env)
{
    PVideoFrame src = child->GetFrame(n, env);
    PVideoFrame dst = has_at_least_v8 ? env->NewVideoFrameP(vi, &src) : env->NewVideoFrame(vi);

    (this->*filter_ptr)(src, dst, env);

    return dst;
}

AVSValue __cdecl Create_vsLGhost(AVSValue args, void* user_data, IScriptEnvironment* env)
{
    enum class Args_num
    {
        Clip = 0,
        Mode,
        Shift,
        Intensity,
        Y,
        U,
        V,
        Opt
    };

    const int mode_num = args[static_cast<int>(Args_num::Mode)].ArraySize();
    std::vector<int> mode(mode_num);
    const int shif_num = args[static_cast<int>(Args_num::Shift)].ArraySize();
    std::vector<int> shift(shif_num);
    const int intensity_num = args[static_cast<int>(Args_num::Intensity)].ArraySize();
    std::vector<int> intensity(intensity_num);

    for (int i = 0; i < mode_num; ++i)
        mode[i] = args[static_cast<int>(Args_num::Mode)][i].AsInt();

    for (int i = 0; i < shif_num; ++i)
        shift[i] = args[static_cast<int>(Args_num::Shift)][i].AsInt();

    for (int i = 0; i < intensity_num; ++i)
        intensity[i] = args[static_cast<int>(Args_num::Intensity)][i].AsInt();

    const int u = args[static_cast<int>(Args_num::U)].AsInt(2);

    return new vsLGhost(args[0].AsClip(), std::move(mode), std::move(shift), std::move(intensity),
        args[static_cast<int>(Args_num::Y)].AsInt(3), u, args[static_cast<int>(Args_num::V)].AsInt(u),
        args[static_cast<int>(Args_num::Opt)].AsInt(-1), env);
}

const AVS_Linkage* AVS_linkage{};

extern "C" __declspec(dllexport) const char* __stdcall AvisynthPluginInit3(IScriptEnvironment* env, const AVS_Linkage* const vectors)
{
    AVS_linkage = vectors;

    env->AddFunction("vsLGhost", "c[mode]i+[shift]i+[intensity]i+[y]i[u]i[v]i[opt]i", Create_vsLGhost, 0);

    return "vsLGhost";
}
