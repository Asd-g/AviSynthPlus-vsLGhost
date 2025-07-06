#include "VCL2/vectorclass.h"
#include "vsLGhost.h"
#include <cassert>

template<typename pixel_t>
void vsLGhost::filter_avx512(PVideoFrame& src, PVideoFrame& dst, IScriptEnvironment* env) const noexcept
{
    using var_t = std::conditional_t<std::is_integral_v<pixel_t>, int, float>;
    using vec_t = std::conditional_t<std::is_integral_v<pixel_t>, Vec16i, Vec16f>;

    auto load = [](const pixel_t* AVS_RESTRICT srcp) noexcept {
        if constexpr (std::is_same_v<pixel_t, uint8_t>)
            return vec_t().load_16uc(srcp);
        else if constexpr (std::is_same_v<pixel_t, uint16_t>)
            return vec_t().load_16us(srcp);
        else
            return vec_t().load(srcp);
    };

    auto store = [&](const vec_t srcp, const vec_t buffer, pixel_t* AVS_RESTRICT dstp) noexcept {
        if constexpr (std::is_same_v<pixel_t, uint8_t>)
        {
            const auto result =
                compress_saturated_s2u(compress_saturated(srcp + (buffer >> 7), zero_si512()), zero_si512()).get_low().get_low();
            result.store_nt(dstp);
        }
        else if constexpr (std::is_same_v<pixel_t, uint16_t>)
        {
            const auto result = compress_saturated_s2u(srcp + (buffer >> 7), zero_si512()).get_low();
            min(result, peak).store_nt(dstp);
        }
        else
        {
            const auto result = mul_add(buffer, 1.0f / 128.0f, srcp);
            result.store_nt(dstp);
        }
    };

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
            const pixel_t* _srcp = reinterpret_cast<const pixel_t*>(src->GetReadPtr(plane));
            pixel_t* AVS_RESTRICT dstp = reinterpret_cast<pixel_t*>(dst->GetWritePtr(plane));
            var_t* AVS_RESTRICT buffer_row = reinterpret_cast<var_t*>(buffer_plane[i].get());

            const int regularPart = (width - 1) & ~(vec_t().size() - 1);

            for (int y = 0; y < height; y++)
            {
                std::memset(buffer_row, 0, width * sizeof(var_t));

                for (auto&& it : options[i][0])
                {
                    const vec_t intensity = static_cast<var_t>(it.intensity);
                    int x = 0;

                    if (it.endX - it.startX >= vec_t().size())
                    {
                        for (x = it.startX; x - it.shift + 1 <= regularPart; x += vec_t().size())
                        {
                            const vec_t buffer = vec_t().load(buffer_row + x);
                            const vec_t result = buffer + (load(_srcp + x - it.shift + 1) - load(_srcp + x - it.shift)) * intensity;
                            result.store(buffer_row + x);
                        }
                    }

                    for (; x < it.endX; x++)
                        buffer_row[x] += (_srcp[x - it.shift + 1] - _srcp[x - it.shift]) * it.intensity;
                }

                for (auto&& it : options[i][1])
                {
                    const vec_t intensity = static_cast<var_t>(it.intensity);
                    int x = 0;

                    if (it.endX - it.startX >= vec_t().size())
                    {
                        for (x = it.startX; x - it.shift <= regularPart; x += vec_t().size())
                        {
                            const vec_t buffer = vec_t().load(buffer_row + x);
                            const vec_t result = buffer + load(_srcp + x - it.shift) * intensity;
                            result.store(buffer_row + x);
                        }
                    }

                    for (; x < it.endX; x++)
                        buffer_row[x] += _srcp[x - it.shift] * it.intensity;
                }

                for (auto&& it : options[i][2])
                {
                    const vec_t intensity = static_cast<var_t>(it.intensity);
                    int x = 0;

                    if (it.endX - it.startX >= vec_t().size())
                    {
                        for (x = it.startX; x - it.shift + 1 <= regularPart; x += vec_t().size())
                        {
                            const vec_t buffer = vec_t().load(buffer_row + x);
                            const vec_t tempEdge = load(_srcp + x - it.shift + 1) - load(_srcp + x - it.shift);
                            const vec_t result = select(tempEdge > 0, buffer + tempEdge * intensity, buffer);
                            result.store(buffer_row + x);
                        }
                    }

                    for (; x < it.endX; x++)
                        if (const var_t tempEdge = _srcp[x - it.shift + 1] - _srcp[x - it.shift]; tempEdge > 0)
                            buffer_row[x] += tempEdge * it.intensity;
                }

                for (auto&& it : options[i][3])
                {
                    const vec_t intensity = static_cast<var_t>(it.intensity);
                    int x = 0;

                    if (it.endX - it.startX >= vec_t().size())
                    {
                        for (x = it.startX; x - it.shift + 1 <= regularPart; x += vec_t().size())
                        {
                            const vec_t buffer = vec_t().load(buffer_row + x);
                            const vec_t tempEdge = load(_srcp + x - it.shift + 1) - load(_srcp + x - it.shift);
                            const vec_t result = select(tempEdge < 0, buffer + tempEdge * intensity, buffer);
                            result.store(buffer_row + x);
                        }
                    }

                    for (; x < it.endX; x++)
                        if (const var_t tempEdge = _srcp[x - it.shift + 1] - _srcp[x - it.shift]; tempEdge < 0)
                            buffer_row[x] += tempEdge * it.intensity;
                }

                for (int x = 0; x < width; x += vec_t().size())
                {
                    const vec_t srcp = load(_srcp + x);
                    const vec_t buffer = vec_t().load_a(buffer_row + x);
                    store(srcp, buffer, dstp + x);
                }

                _srcp += src->GetPitch(plane) / sizeof(pixel_t);
                dstp += dst->GetPitch(plane) / sizeof(pixel_t);
            }
        }
        else if (process[i] == 2)
            env->BitBlt(dst->GetWritePtr(plane), dst->GetPitch(plane), src->GetReadPtr(plane), src->GetPitch(plane), src->GetRowSize(plane),
                height);
    }
}

template void vsLGhost::filter_avx512<uint8_t>(PVideoFrame& src, PVideoFrame& dst, IScriptEnvironment* env) const noexcept;
template void vsLGhost::filter_avx512<uint16_t>(PVideoFrame& src, PVideoFrame& dst, IScriptEnvironment* env) const noexcept;
template void vsLGhost::filter_avx512<float>(PVideoFrame& src, PVideoFrame& dst, IScriptEnvironment* env) const noexcept;
