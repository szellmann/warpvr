// This file is distributed under the MIT license.
// See the LICENSE file for details.

#include <cstring>
#include <exception>
#include <fstream>
#include <iostream>
#include <memory>
#include <ostream>

#include <boost/filesystem.hpp>

#include <GL/glew.h>

#include <Support/CmdLine.h>
#include <Support/CmdLineUtil.h>

#include <visionaray/detail/platform.h>

#include <visionaray/math/math.h>

#include <visionaray/texture/texture.h>

#include <visionaray/aligned_vector.h>
#include <visionaray/bvh.h>
#include <visionaray/pinhole_camera.h>
#include <visionaray/scheduler.h>
#include <visionaray/traverse.h>

#include <common/async/connection.h>
#include <common/async/connection_manager.h>
#include <common/manip/arcball_manipulator.h>
#include <common/manip/pan_manipulator.h>
#include <common/manip/zoom_manipulator.h>
#include <common/timer.h>
#include <common/viewer_glut.h>

#include <vkt/InputStream.hpp>
#include <vkt/LookupTable.hpp>
#include <vkt/RawFile.hpp>
#include <vkt/StructuredVolume.hpp>

#include "common.h"
#include "sphere_renderer.h"

using namespace visionaray;

using namespace boost::placeholders;

using viewer_type   = viewer_glut;


//-------------------------------------------------------------------------------------------------
// I/O utility for camera lookat only - not fit for the general case!
//

std::istream& operator>>(std::istream& in, pinhole_camera& cam)
{
    vec3 eye;
    vec3 center;
    vec3 up;

    in >> eye >> std::ws >> center >> std::ws >> up >> std::ws;
    cam.look_at(eye, center, up);

    return in;
}

std::ostream& operator<<(std::ostream& out, pinhole_camera const& cam)
{
    out << cam.eye() << '\n';
    out << cam.center() << '\n';
    out << cam.up() << '\n';
    return out;
}


struct client
{
    async::connection_manager_pointer manager;
    async::connection_pointer conn;
    pinhole_camera camera;
    pinhole_camera old_camera;
    std::mutex mutex;

    aligned_vector<vec4> spheres;
    aligned_vector<vec4> sphere_colors;

    int width = 0;
    int height = 0;
    bool newData = false;

    client()
        : manager(async::make_connection_manager())
    {
    }

    void resize(int w, int h)
    {
        width = w;
        height = h;

        std::unique_lock<std::mutex> l(mutex);
        spheres.resize(w * h);
        sphere_colors.resize(w * h);
    }

    void connect(std::string const& host = "localhost", unsigned short port = 31050)
    {
        std::cout << "client: connecting...\n";

        manager->connect(host, port, boost::bind(&client::handle_new_connection, this, _1, _2));
    }

    void run()
    {
        manager->run_in_thread();
    }

    void wait()
    {
        manager->wait();
    }

    void handle_message(async::connection::reason reason, async::message_pointer message, boost::system::error_code const& e)
    {
        if (e)
        {
            std::cerr << "client: error: " << e.message() << '\n';

            manager->stop();
            return;
        }

        if (reason == async::connection::Read)
        {
            if (message->type() == MtPointCloud)
            {
                std::unique_lock<std::mutex> l(mutex);
                memcpy((char*)spheres.data(), message->data(), width * height * sizeof(vec4));
            }
            else if (message->type() == MtColors)
            {
                {
                    std::unique_lock<std::mutex> l(mutex);
                    memcpy((char*)sphere_colors.data(), message->data(), width * height * sizeof(vec4));
                    newData = true;
                }

                // reinitiate rendering
                pinhole_camera cam;
                {
                    while (camera == old_camera)
                    {
                        // wait
                    }
                    std::unique_lock<std::mutex> l(mutex);
                    old_camera = camera;
                    cam = camera;
                }
                conn->write(MtCamera, (char*)&cam, (char*)&cam + sizeof(cam));
            }
        }
        else
        {
        }
    }

    bool handle_new_connection(async::connection_pointer new_conn, boost::system::error_code const& e)
    {
        if (e)
        {
            std::cerr << "client: could not connect to server: " << e.message() << '\n';

            manager->stop();
            return false;
        }

        std::cout << "client: connected\n";

        // Accept and save this connection
        // and set the messange handler of the connection
        conn = new_conn;
        conn->set_handler(boost::bind(&client::handle_message, this, _1, _2, _3));

        // initiate rendering
        pinhole_camera cam;
        {
            std::unique_lock<std::mutex> l(mutex);
            cam = camera;
        }
        std::cout << cam.get_viewport() << '\n';
        conn->write(MtCamera, (char*)&cam, (char*)&cam + sizeof(cam));

        return true;
    }
};


struct renderer : viewer_type
{
    using host_ray_type = basic_ray<float>;

    renderer(client& cli, int argc, char** argv)
        : viewer_type(512, 512, "Client")
        , cli(cli)
    {
        bbox = aabb({0,0,0}, {256,256,128});
    }

    client& cli;
    sphere_renderer rend;
    aabb                                        bbox;
    pinhole_camera                              cam;

    aligned_vector<vec4>                        sphere_colors;

    bool measure_bvh_performance = false;
    bool measure_performance = false;

protected:

    void on_display();
    void on_key_press(visionaray::key_event const& event);
    void on_mouse_move(visionaray::mouse_event const& event);
    void on_mouse_up(visionaray::mouse_event const& event);
    void on_resize(int w, int h);

};


void renderer::on_display()
{
    if (cli.newData)
    {
        std::unique_lock<std::mutex> l(cli.mutex);

        if (measure_bvh_performance)
        {
            int N = 100;
            double sec = 0.0;
            for (int i = 0; i < N; ++i)
            {
                timer t;
                rend.reset(cli.spheres.data(), cli.sphere_colors.data(), cli.spheres.size());
                sec += t.elapsed();
            }
            std::cout << "LBVH: " << (sec / N) << '\n';
        }
        else
            rend.reset(cli.spheres.data(), cli.sphere_colors.data(), cli.spheres.size());

        cli.newData = false;
    }

    auto bgcolor = background_color();
    glClearColor(bgcolor.x, bgcolor.y, bgcolor.z, 1.0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);


#if 0
    glMatrixMode(GL_MODELVIEW);
    glLoadMatrixf(cam.get_view_matrix().data());

    glMatrixMode(GL_PROJECTION);
    glLoadMatrixf(cam.get_proj_matrix().data());

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDisable(GL_LIGHTING);
    glPointSize(1);
    glBegin(GL_POINTS);
    for (int i=0; i<cli.spheres.size(); ++i)
    {
        if (cli.spheres[i].w > 0.f)
        {
        glColor4f(cli.sphere_colors[i].x,
                  cli.sphere_colors[i].y,
                  cli.sphere_colors[i].z,
                  cli.sphere_colors[i].w);

        glVertex3f(cli.spheres[i].x,
                   cli.spheres[i].y,
                   cli.spheres[i].z);
        }
    }
    glEnd();
#else
    if (measure_performance)
    {
        int N = 100;
        double sec = 0.0;
        for (int i = 0; i < N; ++i)
        {
            timer t;
            rend.render(cam);
            sec += t.elapsed();
        }
        std::cout << "FPS: " << N/sec << '\n';
    }
    else
        rend.render(cam);
#endif
}

void renderer::on_key_press(visionaray::key_event const& event)
{
    static const std::string camera_file_base = "client-camera";
    static const std::string camera_file_suffix = ".txt";

    switch (event.key())
    {

    case 'b':
        measure_bvh_performance = !measure_bvh_performance;
        break;

    case 'm':
        measure_performance = !measure_performance;
        break;

    case 'u':
        {
            int inc = 0;
            std::string inc_str = "";

            std::string filename = camera_file_base + inc_str + camera_file_suffix;

            while (boost::filesystem::exists(filename))
            {
                ++inc;
                inc_str = std::to_string(inc);

                while (inc_str.length() < 4)
                {
                    inc_str = std::string("0") + inc_str;
                }

                inc_str = std::string("-") + inc_str;

                filename = camera_file_base + inc_str + camera_file_suffix;
            }

            std::ofstream file(filename);
            if (file.good())
            {
                std::cout << "Storing camera to file: " << filename << '\n';
                file << cam;
            }
        }
        break;

    case 'v':
        {
            std::string filename = camera_file_base + camera_file_suffix;

            std::ifstream file(filename);
            if (file.good())
            {
                file >> cam;
                std::cout << "Load camera from file: " << filename << '\n';
            }
        }
        break;
    }

    viewer_type::on_key_press(event);
}

void renderer::on_mouse_move(visionaray::mouse_event const& event)
{
    if (event.buttons() != visionaray::mouse::NoButton)
    {
        std::unique_lock<std::mutex> l(cli.mutex);
        cli.camera = cam;
    }

    viewer_type::on_mouse_move(event);
}

void renderer::on_mouse_up(visionaray::mouse_event const& event)
{
    if (event.buttons() != visionaray::mouse::NoButton)
    {
        std::unique_lock<std::mutex> l(cli.mutex);
        cli.camera = cam;
    }

    viewer_type::on_mouse_up(event);
}


//-------------------------------------------------------------------------------------------------
// resize event
//

void renderer::on_resize(int w, int h)
{
    cam.set_viewport(0, 0, w, h);
    float aspect = w / static_cast<float>(h);
    cam.perspective(45.0f * constants::degrees_to_radians<float>(), aspect, 0.001f, 1000.0f);
    rend.resize(w, h);

    viewer_type::on_resize(w, h);
}


int main(int argc, char** argv)
{
    client cli;

    renderer rend(cli, argc, argv);

    try
    {
        rend.init(argc, argv);
    }
    catch (std::exception const& e)
    {
        std::cerr << e.what() << '\n';
        return EXIT_FAILURE;
    }

    float aspect = rend.width() / static_cast<float>(rend.height());

    rend.cam.set_viewport(0, 0, rend.width(), rend.height());
    rend.cam.perspective(45.0f * constants::degrees_to_radians<float>(), aspect, 0.001f, 1000.0f);
    rend.cam.view_all( rend.bbox );

    rend.add_manipulator( std::make_shared<arcball_manipulator>(rend.cam, mouse::Left) );
    rend.add_manipulator( std::make_shared<pan_manipulator>(rend.cam, mouse::Middle) );
    // Additional "Alt + LMB" pan manipulator for setups w/o middle mouse button
    rend.add_manipulator( std::make_shared<pan_manipulator>(rend.cam, mouse::Left, keyboard::Alt) );
    rend.add_manipulator( std::make_shared<zoom_manipulator>(rend.cam, mouse::Right) );

    cli.camera = rend.cam;
    cli.resize(rend.width(), rend.height());
    cli.connect("localhost", 31050);
    cli.run();

    rend.event_loop();
}
