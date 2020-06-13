// This file is distributed under the MIT license.
// See the LICENSE file for details.

#pragma once

#include <memory>

#include <visionaray/math/math.h>

namespace visionaray
{

class sphere_renderer
{
public:
    sphere_renderer();
   ~sphere_renderer();

    void reset(const vec4* spheres, const vec4* colors, size_t num_spheres);

    void resize(int w, int h);

    void render(pinhole_camera const& cam);

private:
    struct impl;
    std::unique_ptr<impl> impl_;
};

} // visionaray
