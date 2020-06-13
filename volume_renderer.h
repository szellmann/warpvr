// This file is distributed under the MIT license.
// See the LICENSE file for details.

#pragma once

#include <cstdint>
#include <string>

#include <thrust/device_vector.h>

#include <visionaray/math/math.h>
#include <visionaray/texture/texture.h>
#include <visionaray/aligned_vector.h>
#include <visionaray/pinhole_camera.h>
#include <visionaray/simple_gpu_buffer_rt.h>
#include <visionaray/scheduler.h>

namespace visionaray
{

class volume_renderer
{
public:
    volume_renderer(std::string filename);

    void resize(int w, int h);

    void render(pinhole_camera const& cam);

    vec4 const* color_buffer() const;
    vec4 const* object_space_samples() const;

    size_t num_samples() const;

    std::string filename;

private:
    aabb bbox;
    pinhole_camera cam;

#ifdef __CUDACC__
    simple_gpu_buffer_rt<PF_RGBA8, PF_UNSPECIFIED> device_rt;
    cuda_sched<basic_ray<float>> device_sched;
#endif

    // volkit
    vkt::StructuredVolume volume;
    vkt::LookupTable lut;

    // texture references

#ifdef __CUDACC__
    cuda_texture<uint8_t, 3> device_volume;
    cuda_texture<vec4, 1> device_transfunc;
#endif

    thrust::device_vector<vec4> device_spheres;
    thrust::device_vector<vec4> device_sphere_colors;

    thrust::host_vector<vec4> host_spheres;
    thrust::host_vector<vec4> host_sphere_colors;
};

} // visionaray
