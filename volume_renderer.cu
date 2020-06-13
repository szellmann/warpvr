// This file is distributed under the MIT license.
// See the LICENSE file for details.

#include <cassert>

#include <thrust/copy.h>

#include <visionaray/detail/platform.h>

#include <visionaray/math/math.h>

#include <vkt/InputStream.hpp>
#include <vkt/LookupTable.hpp>
#include <vkt/RawFile.hpp>
#include <vkt/StructuredVolume.hpp>

#include <common/timer.h>

#include "volume_renderer.h"

namespace visionaray
{

volume_renderer::volume_renderer(std::string filename)
    : filename(filename)
{
    vkt::RawFile file(filename.c_str(), "r");
    vkt::Vec3i dims = file.getDims();
    uint16_t bpv = file.getBytesPerVoxel();

    volume = vkt::StructuredVolume(dims.x, dims.y, dims.z, bpv);
    vkt::InputStream is(file);
    is.read(volume);

    float rgba[] = {
        //1.f, 1.f, 1.f, .005f,
        //1.f, 1.f, 1.f, 0.f,
        0.f, 0.f, .2f, .005f,
        0.f, .1f, .1f, .25f,
        .5f, .5f, .7f, .5f,
        .7f, .7f, .07f, .75f,
        1.f, .3f, .3f, 1.f
    };
    lut = vkt::LookupTable(5,1,1,vkt::ColorFormat::RGBA32F);
    lut.setData((uint8_t*)rgba);

    vkt::Vec3f dist = volume.getDist();
    bbox = aabb(
            { 0.f, 0.f, 0.f },
            { dims.x * dist.x, dims.y * dist.y, dims.z * dist.z }
            );

    device_volume = cuda_texture<uint8_t, 3>((size_t)dims.x, (size_t)dims.y, (size_t)dims.z);
    device_volume.reset(volume.getData());
    device_volume.set_filter_mode(Nearest);
    device_volume.set_address_mode(Clamp);

    device_transfunc = cuda_texture<vec4, 1>((size_t)lut.getDims().x);
    device_transfunc.reset((vec4*)lut.getData());
    device_transfunc.set_filter_mode(Linear);
    device_transfunc.set_address_mode(Clamp);
}

void volume_renderer::resize(int w, int h)
{
    std::cout << w << ' ' << h << '\n';
    device_rt.resize(w, h);

    device_spheres.resize(w * h);
    device_sphere_colors.resize(w * h);

    host_spheres.resize(w * h);
    host_sphere_colors.resize(w * h);
}

// Stolen from Ingo's owl
/*! simple 24-bit linear congruence generator */
template<unsigned int N=4>
struct LCG {

  inline __device__ LCG()
  { /* intentionally empty so we can use it in device vars that
       don't allow dynamic initialization (ie, PRD) */
  }
  inline __device__ LCG(unsigned int val0, unsigned int val1)
  { init(val0,val1); }

  inline __device__ void init(unsigned int val0, unsigned int val1)
  {
    unsigned int v0 = val0;
    unsigned int v1 = val1;
    unsigned int s0 = 0;

    for (unsigned int n = 0; n < N; n++) {
      s0 += 0x9e3779b9;
      v0 += ((v1<<4)+0xa341316c)^(v1+s0)^((v1>>5)+0xc8013ea4);
      v1 += ((v0<<4)+0xad90777d)^(v0+s0)^((v0>>5)+0x7e95761e);
    }
    state = v0;
  }

  // Generate random unsigned int in [0, 2^24)
  inline __device__ float operator() ()
  {
    const uint32_t LCG_A = 1664525u;
    const uint32_t LCG_C = 1013904223u;
    state = (LCG_A * state + LCG_C);
    return ldexpf(float(state), -32);
    // return (state & 0x00FFFFFF) / (float) 0x01000000;
  }

  uint32_t state;
};

struct kernel
{
    using S = float;

    __device__
    result_record<S> operator()(basic_ray<float> ray, int x, int y)
    {
        LCG<4> rnd(x,y);
        result_record<S> result;

        auto hit_rec = intersect(ray, bbox);
        auto t = hit_rec.tnear;

        result.color = vec4(0.0);

        spheres[y * width + x] = vec4(0.f);
        sphere_colors[y * width + x] = vec4(0.f);

    
        int it = 0;
        vec4 oldColor(0.f);
        bool dont = false;
        while (t < hit_rec.tfar)
        {
            auto pos = ray.ori + ray.dir * t;
            auto tex_coord = pos / vector<3, S>(bbox.size());

            // sample volume and do post-classification
            float voxel = convert_to_float(tex3D(volume_ref, tex_coord)) / S(255.f);
            vec4 color = tex1D(transfunc_ref, voxel);

            // opacity correction
            color.w = S(1.f) - pow(S(1.f) - color.w, S(dt));

            // premultiplied alpha
            color.xyz() *= color.w;

            // front-to-back alpha compositing
            result.color += select(
                    t < hit_rec.tfar,
                    color * (1.0f - result.color.w),
                    vec4(0.0)
                    );

            //if (!dont && (it == 0 || color.w > 0.1f))
            if (!dont && result.color.w > 0.8f)
            {
                spheres[y * width + x] = vec4(pos, .5f);
                dont = true;
            }

            ++it;

            // early-ray termination - don't traverse w/o a contribution
            if ( all(result.color.w >= 0.999) )
            {
                break;
            }

            // step on
            t += dt;
            oldColor = color;
        }

        if (!dont && result.color.w > 1e-8f)
        {
            auto pos = ray.ori + ray.dir * hit_rec.tfar;
            //auto pos = ray.ori + ray.dir * lerp(hit_rec.tnear,hit_rec.tfar,rnd());
            spheres[y * width + x] = vec4(pos, .5f);
        }

        sphere_colors[y * width + x] = result.color;

        result.hit = hit_rec.hit;
        return result;
    }

    vec4* spheres;
    vec4* sphere_colors;
    cuda_texture_ref<uint8_t, 3> volume_ref;
    cuda_texture_ref<vec4, 1> transfunc_ref;
    aabb bbox;
    int width;
    float dt;
};

void volume_renderer::render(pinhole_camera const& cam)
{
    // some setup

    using R = basic_ray<float>;
    using S = R::scalar_type;
    using C = vector<4, S>;

    auto sparams = make_sched_params(cam, device_rt);

    // call kernel in schedulers' frame() method

    kernel kern;
    kern.spheres = thrust::raw_pointer_cast(device_spheres.data());
    kern.sphere_colors = thrust::raw_pointer_cast(device_sphere_colors.data());
    kern.volume_ref = cuda_texture_ref<uint8_t, 3>(device_volume);
    kern.transfunc_ref = cuda_texture_ref<vec4, 1>(device_transfunc);
    kern.bbox = bbox;
    kern.width = device_rt.width();
    kern.dt = .5f;

    cuda::timer t;
    device_sched.frame(kern, sparams);
    std::cout << "Rendered. Elapsed: " << t.elapsed() << '\n';

    thrust::copy(device_spheres.begin(), device_spheres.end(), host_spheres.begin());
    thrust::copy(device_sphere_colors.begin(), device_sphere_colors.end(), host_sphere_colors.begin());
}

vec4 const* volume_renderer::color_buffer() const
{
    return host_sphere_colors.data();
}

vec4 const* volume_renderer::object_space_samples() const
{
    return host_spheres.data();
}

size_t volume_renderer::num_samples() const
{
    assert(host_sphere_colors.size() == host_spheres.size());

    return host_spheres.size();
}

} // visionaray
