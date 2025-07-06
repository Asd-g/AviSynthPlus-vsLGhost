#pragma once

#include <array>
#include <cstddef>
#include <memory>
#include <vector>

#include "avisynth.h"

AVS_FORCEINLINE void* aligned_malloc(size_t size, size_t align) noexcept
{
    void* result = [&]() noexcept {
#ifdef _WIN32
        return _aligned_malloc(size, align);
#else
        if (posix_memalign(&result, align, size))
            return result = nullptr;
        else
            return result;
#endif
    }();

    return result;
}

AVS_FORCEINLINE void aligned_free(void* ptr) noexcept
{
#ifdef _WIN32
    _aligned_free(ptr);
#else
    free(ptr);
#endif
}

template<typename T>
struct aligned_array_deleter
{
    void operator()(T* ptr) const noexcept
    {
        if (ptr)
            aligned_free(ptr);
    }
};

template<typename T>
using aligned_unique_ptr = std::unique_ptr<T[], aligned_array_deleter<T>>;

template<typename T>
inline aligned_unique_ptr<T> make_unique_aligned_array(size_t num_elements, size_t alignment) noexcept
{
    if (num_elements == 0)
        return aligned_unique_ptr<T>(nullptr);

    T* ptr = static_cast<T*>(aligned_malloc(num_elements * sizeof(T), alignment));

    return aligned_unique_ptr<T>(ptr);
}

struct OptionData final
{
    int shift;
    int intensity;
    int startX;
    int endX;
};

class vsLGhost : public GenericVideoFilter
{
    const std::vector<int> mode_;
    std::vector<int> shift_;
    const std::vector<int> intensity_;
    const std::array<int, 3> process;
    std::vector<OptionData> options[3][4];
    const int peak;
    const bool has_at_least_v8;
    const int planecount;
    const std::array<int, 3> plane_width;
    mutable std::array<aligned_unique_ptr<std::byte>, 3> buffer_plane;

    template<typename pixel_t>
    void filter_c(PVideoFrame& src, PVideoFrame& dst, IScriptEnvironment* env) const noexcept;
    template<typename pixel_t>
    void filter_sse2(PVideoFrame& src, PVideoFrame& dst, IScriptEnvironment* env) const noexcept;
    template<typename pixel_t>
    void filter_avx2(PVideoFrame& src, PVideoFrame& dst, IScriptEnvironment* env) const noexcept;
    template<typename pixel_t>
    void filter_avx512(PVideoFrame& src, PVideoFrame& dst, IScriptEnvironment* env) const noexcept;

    void (vsLGhost::*filter_ptr)(PVideoFrame& src, PVideoFrame& dst, IScriptEnvironment* env) const noexcept;

public:
    vsLGhost(PClip _child, std::vector<int> mode, std::vector<int> shift, std::vector<int> intensity, int y, int u, int v, int opt,
        IScriptEnvironment* env);
    PVideoFrame __stdcall GetFrame(int n, IScriptEnvironment* env) override;
    int __stdcall SetCacheHints(int cachehints, int frame_range) noexcept override
    {
        return cachehints == CACHE_GET_MTMODE ? MT_MULTI_INSTANCE : 0;
    }
};
