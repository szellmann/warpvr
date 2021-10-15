// This file is distributed under the MIT license.
// See the LICENSE file for details.

#include <thrust/device_vector.h>

#include <visionaray/aligned_vector.h>
#include <visionaray/bvh.h>
#include <visionaray/pinhole_camera.h>
#include <visionaray/pixel_unpack_buffer_rt.h>
#include <visionaray/result_record.h>
#include <visionaray/scheduler.h>
#include <visionaray/traverse.h>

#include <common/timer.h>

#include "sphere_renderer.h"

namespace visionaray
{

//-------------------------------------------------------------------------------------------------
// Solid sphere, one that a ray can travel through from tnear to tfar
//

struct solid_sphere : basic_sphere<float>
{
};

template <typename S>
struct solid_hit_record : visionaray::hit_record<basic_ray<S>, primitive<unsigned>>
{
    S tfar;
    S radius2;
};

template <typename S, typename Cond>
__device__
void update_if(solid_hit_record<S>& dst, solid_hit_record<S> const& src, Cond const& cond)
{
    dst.hit        |= cond;
    dst.t           = select( cond, src.t, dst.t );
    dst.prim_id     = select( cond, src.prim_id, dst.prim_id );
    dst.geom_id     = select( cond, src.geom_id, dst.geom_id );
    dst.u           = select( cond, src.u, dst.u );
    dst.v           = select( cond, src.v, dst.v );

    dst.tfar        = select( cond, src.tfar , dst.tfar  );
    dst.radius2     = select( cond, src.radius2 , dst.radius2  );
}

void split_primitive(aabb& L, aabb& R, float plane, int axis, solid_sphere const& prim)
{
    split_primitive(L, R, plane, axis, static_cast<basic_sphere<float>>(prim));
}


template <typename S>
__device__
solid_hit_record<S> intersect(
        basic_ray<S> const& ray,
        solid_sphere const& sphere
        )
{
    typedef basic_ray<S> ray_type;
    typedef vector<3, S> vec_type;

    ray_type r = ray;
    r.ori -= vec_type( sphere.center );

    auto A = dot(r.dir, r.dir);
    auto B = dot(r.dir, r.ori) * S(2.0);
    auto C = dot(r.ori, r.ori) - sphere.radius * sphere.radius;

    // solve Ax**2 + Bx + C
    auto disc = B * B - S(4.0) * A * C;
    auto valid = disc >= S(0.0);

    auto root_disc = select(valid, sqrt(disc), disc);

    auto q = select( B < S(0.0), S(-0.5) * (B - root_disc), S(-0.5) * (B + root_disc) );

    auto tnear = q / A;
    auto tfar = C / q;

    auto mask = tnear > tfar;
    auto tmp = select(mask, tnear, S(0.0));
    tnear = select(mask, tfar, tnear);
    tfar = select(mask, tmp, tfar);

    valid &= tnear > S(0.0);

    solid_hit_record<S> result;
    result.hit = valid;
    result.prim_id = sphere.prim_id;
    result.geom_id = sphere.geom_id;
    result.t = select( valid, tnear, S(-1.0) );
    result.tfar = select( valid, tfar, S(-1.0) );
    result.radius2 = select( valid, sphere.radius + sphere.radius, S(-1.0) );
    return result;
}


struct render_kernel
{
    __device__
    result_record<float> operator()(basic_ray<float> ray, int x, int y)
    {
        result_record<float> result;
        result.color = vec4(0.0);

#ifdef __CUDA_ARCH__
        // Perform multi-hit, we allow for up to 16 hits
        // Multi-hit returns a sorted array (based on
        // ray parameter "t") of hit records
        auto hit_rec = closest_hit(ray, bvhs, bvhs + 1);

        // Use closest hit in the sequence
        // for visibility testing
        result.hit = hit_rec.hit;
        if (hit_rec.hit)
        {
            result.color = sphere_colors[hit_rec.prim_id];
            result.isect_pos  = ray.ori + ray.dir * hit_rec.t;
        }
#endif

        return result;
    }

    cuda_index_bvh<solid_sphere>::bvh_ref* bvhs;
    vec4* sphere_colors;
};


struct sphere_renderer::impl
{
    thrust::device_vector<vec4> device_sphere_colors;
    pixel_unpack_buffer_rt<PF_RGBA8, PF_UNSPECIFIED> host_rt;
    cuda_sched<basic_ray<float>> host_sched;
    cuda_index_bvh<solid_sphere> host_bvh;
};

sphere_renderer::sphere_renderer()
    : impl_(new impl)
{
}

sphere_renderer::~sphere_renderer()
{
}

void sphere_renderer::reset(const vec4* spheres, const vec4* colors, size_t num_spheres)
{
    std::vector<vec4> sphere_colors;

    std::vector<solid_sphere> sphere_geom;

    unsigned prim_id = 0;
    for (size_t i = 0; i < num_spheres; ++i)
    {
        if (spheres[i].w > 0.f)
        {
            solid_sphere sp;
            sp.center = spheres[i].xyz();
            sp.radius = spheres[i].w;
            sphere_geom.push_back(sp);
            sphere_geom.back().prim_id = prim_id++;
            sphere_colors.push_back(colors[i]);
        }
    }

    thrust::device_vector<solid_sphere> d_sphere_geom(sphere_geom);
    impl_->device_sphere_colors.resize(sphere_colors.size());
    thrust::copy(sphere_colors.begin(), sphere_colors.end(), impl_->device_sphere_colors.begin());

    lbvh_builder builder;

    cuda::timer t;
    impl_->host_bvh = builder.build(cuda_index_bvh<solid_sphere>{},
                                    thrust::raw_pointer_cast(d_sphere_geom.data()),
                                    d_sphere_geom.size());
    std::cout << t.elapsed() << '\n';
}

void sphere_renderer::resize(int w, int h)
{
    impl_->host_rt.resize(w, h);
}

void sphere_renderer::render(pinhole_camera const& cam)
{
    if (impl_->device_sphere_colors.empty())
        return;

    // some setup

    auto sparams = make_sched_params(
            cam,
            impl_->host_rt
            );

    using bvh_ref = cuda_index_bvh<solid_sphere>::bvh_ref;

    thrust::device_vector<bvh_ref> bvhs;
    bvhs.push_back(impl_->host_bvh.ref());

    render_kernel kern;
    kern.bvhs = thrust::raw_pointer_cast(bvhs.data());
    kern.sphere_colors = thrust::raw_pointer_cast(impl_->device_sphere_colors.data());

    impl_->host_sched.frame(kern, sparams);

    // display the rendered image
    impl_->host_rt.display_color_buffer();
}

} // visionaray
