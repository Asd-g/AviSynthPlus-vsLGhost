## Description

Luminance Ghost reduction filter. Can be used for removing luminance ghost or edge ghost (ringing).

This is [a port of the VapourSynth plugin LGhost](https://github.com/HomeOfVapourSynthEvolution/VapourSynth-LGhost).

### Requirements:

- AviSynth+ 3.6 or later

- Microsoft VisualC++ Redistributable Package 2022 (can be downloaded from [here](https://github.com/abbodi1406/vcredist/releases))

### Usage:

```
vsLGhost (clip, int[] mode, int[] shift, int[] intensity, int "y", int "u", int "v", int "opt")
```

### Parameters:

- clip\
    A clip to process. All planar formats are supported.

- mode\
    Ghost removal mode.\
    1: Edge.\
    2: Luminance.\
    3: Rising edge.\
    4: Falling edge.

- shift\
    Width to shift.\
    Must be between -width and width (exclusive).

- intensity\
    Strength.\
    Must not be 0 and must be between -128 and 127 (inclusive).

- y, u, v\
    Planes to process.\
    1: Return garbage.\
    2: Copy plane.\
    3: Process plane. Always process planes when the clip is RGB.\
    Default: y = 3; u = v = 2.

- opt\
    Sets which cpu optimizations to use.\
    -1: Auto-detect.\
    0: Use C++ code.\
    1: Use SSE2 code.\
    2: Use AVX2 code.\
    3: Use AVX512 code.\
    Default: -1.

Each ghost consists of individual value from mode, shift and intensity. For example, vsLGhost(mode=[2, 2, 1, 1], shift=[4, 7, -4, -7], intensity=[20, 10, -15, -5]) corresponds to four ghosts. The first ghost is (mode=2, shift=4, intensity=20), the second ghost is (mode=2, shift=7, intensity=10), and so on.

### Building:

#### Prerequisites
- **Git**
- **CMake** >= 3.28
- A **C++20 capable compiler** (e.g., Visual Studio 2022, GCC 11+, Clang 12+)

1.  Clone the repository:

    ```
    git clone --depth 1 --shallow-submodules --recursive https://github.com/Asd-g/AviSynthPlus-vsLGhost
    cd AviSynthPlus-vsLGhost
    ```

2.  Configure and build the project:

    ```
    cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH=<path_to_avs_headers>
    cmake --build build -j$(nproc)
    ```

3.  (Linux) Install the plugin (optional):

    ```
    sudo cmake --install build
    ```
