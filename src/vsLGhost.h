#pragma once

#include <memory>
#include <vector>

#include "avisynth.h"

using unique_float = std::unique_ptr<float[], decltype(&_aligned_free)>;

struct OptionData final
{
    int shift, intensity, startX, endX;
};

class vsLGhost : public GenericVideoFilter
{
    std::vector<int> mode_;
    std::vector<int> shift_;
    std::vector<int> intensity_;
    int opt_;
    int process[3];
    std::vector<OptionData> options[3][4];
    int peak;
    bool has_at_least_v8;
    void* buffer;

    template<typename pixel_t>
    void filter_c(PVideoFrame& src, PVideoFrame& dst, const vsLGhost* const __restrict, IScriptEnvironment* env) noexcept;
    template<typename pixel_t>
    void filter_sse2(PVideoFrame& src, PVideoFrame& dst, const vsLGhost* const __restrict, IScriptEnvironment* env) noexcept;
    template<typename pixel_t>
    void filter_avx2(PVideoFrame& src, PVideoFrame& dst, const vsLGhost* const __restrict, IScriptEnvironment* env) noexcept;
    template<typename pixel_t>
    void filter_avx512(PVideoFrame& src, PVideoFrame& dst, const vsLGhost* const __restrict, IScriptEnvironment* env) noexcept;

public:
    vsLGhost(PClip _child, const std::vector<int>& mode, const std::vector<int>& shift, const std::vector<int>& intensity, int y, int u, int v, int opt, IScriptEnvironment* env);
    PVideoFrame __stdcall GetFrame(int n, IScriptEnvironment* env);
    int __stdcall SetCacheHints(int cachehints, int frame_range)
    {
        return cachehints == CACHE_GET_MTMODE ? MT_MULTI_INSTANCE : 0;
    }
    ~vsLGhost();
};