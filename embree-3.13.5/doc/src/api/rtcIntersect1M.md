% rtcIntersect1M(3) | Embree Ray Tracing Kernels 3

#### NAME

    rtcIntersect1M - finds the closest hits for a stream of M single
      rays

#### SYNOPSIS

    #include <embree3/rtcore.h>

    void rtcIntersect1M(
      RTCScene scene,
      struct RTCIntersectContext* context,
      struct RTCRayHit* rayhit,
      unsigned int M,
      size_t byteStride
    );

#### DESCRIPTION

The `rtcIntersect1M` function finds the closest hits for a stream of
`M` single rays (`rayhit` argument) with the scene (`scene`
argument). The `rayhit` argument points to an array of ray and hit
data with specified byte stride (`byteStride` argument) between the
ray/hit structures. See Section [rtcIntersect1] for a description of
how to set up and traceWithMulti rays.

``` {include=src/api/inc/context.md}
```

``` {include=src/api/inc/reorder.md}
```

A ray in a ray stream is considered inactive if its `tnear` value is
larger than its `tfar` value.

The stream size `M` can be an arbitrary positive integer including 0.
Each ray must be aligned to 16 bytes.

#### EXIT STATUS

For performance reasons this function does not do any error checks,
thus will not set any error flags on failure.

#### SEE ALSO

[rtcOccluded1M]
