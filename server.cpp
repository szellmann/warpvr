// This file is distributed under the MIT license.
// See the LICENSE file for details.

#include <exception>
#include <fstream>
#include <iostream>
#include <memory>
#include <ostream>

#include <GL/glew.h>

#include <Support/CmdLine.h>
#include <Support/CmdLineUtil.h>

#include <visionaray/detail/platform.h>

#include <visionaray/math/math.h>

#include <visionaray/texture/texture.h>

#include <visionaray/aligned_vector.h>
#include <visionaray/pinhole_camera.h>
#include <visionaray/simple_buffer_rt.h>
#include <visionaray/scheduler.h>

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
#include "volume_renderer.h"

using namespace visionaray;

using namespace boost::placeholders;

using viewer_type = viewer_glut;

struct renderer : viewer_type
{
    using host_ray_type = basic_ray<float>;

    renderer(int argc, char** argv)
        : viewer_type(0, 0, "Server")
        , volrend(argv[1])
    {
        using namespace support;

        add_cmdline_option( cl::makeOption<std::string&>(
            cl::Parser<>(),
            "filename",
            cl::Desc("Input file in wavefront obj format"),
            cl::Positional,
            cl::Required,
            cl::init(volrend.filename)
            ) );
    }

    void render()
    {
        volrend.render(cam);
    }

    void resize(int w, int h)
    {
        volrend.resize(w, h);
        on_resize(w, h);
    }

    pinhole_camera cam;

    volume_renderer volrend;

protected:

    void on_display();
    void on_key_press(visionaray::key_event const& event);
    void on_resize(int w, int h);

};


void renderer::on_display()
{
}


//-------------------------------------------------------------------------------------------------
// resize event
//

void renderer::on_resize(int w, int h)
{
    viewer_type::resize(w, h);
}

void renderer::on_key_press(visionaray::key_event const& event)
{
    viewer_type::on_key_press(event);
}


struct server
{
    async::connection_manager_pointer manager;
    async::connection_pointer conn;

    renderer& rend;

    explicit server(renderer& rend, unsigned short port = 31050)
        : manager(async::make_connection_manager(port))
        , rend(rend)
    {
    }

    void accept()
    {
        std::cout << "server: accepting...\n";

        manager->accept(boost::bind(&server::handle_new_connection, this, _1, _2));
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
            std::cout << "server: error: " << e.message() << '\n';

            manager->stop();
            return;
        }

        timer t;

        if (reason == async::connection::Read)
        {
            if (message->type() == MtCamera)
            {
                if (message->size() != sizeof(rend.cam))
                {
                    std::cerr << "server: size mismatch while reading\n";
                    return;
                }

                rend.cam = *reinterpret_cast<pinhole_camera*>(message->data());

                recti viewport = rend.cam.get_viewport();
                if (viewport.w != rend.width() || viewport.h != rend.height())
                    rend.resize(viewport.w, viewport.h);

                rend.render();

                char* spheresBegin = (char*)rend.volrend.object_space_samples();
                char* spheresEnd = spheresBegin + rend.volrend.num_samples() * sizeof(vec4);
                t.reset();
                conn->write(MtPointCloud, spheresBegin, spheresEnd);
            }
        }
        else
        {
            if (message->type() == MtPointCloud)
            {
                char* colorsBegin = (char*)rend.volrend.color_buffer();
                char* colorsEnd = colorsBegin + rend.volrend.num_samples() * sizeof(vec4);
                conn->write(MtColors, colorsBegin, colorsEnd);

                std::cout << "Points written. Elapsed: " << t.elapsed() << '\n';
                t.reset();
            }
            else if (message->type() == MtColors)
            {
                std::cout << "Colors written. Elapsed: " << t.elapsed() << '\n';
                t.reset();
            }
        }
    }

    bool handle_new_connection(async::connection_pointer new_conn, boost::system::error_code const& e)
    {
        if (e)
        {
            std::cerr << "server: could not connect to client: " << e.message() << '\n';

            manager->stop();
            return false;
        }

        std::cout << "server: connected\n";

        // Accept and save this connection
        // and set the message handler of the connection
        conn = new_conn;
        conn->set_handler(boost::bind(&server::handle_message, this, _1, _2, _3));

        return true;
    }
};


int main(int argc, char** argv)
{
    renderer rend(argc, argv);

    try
    {
        rend.init(argc, argv);
    }
    catch (std::exception const& e)
    {
        std::cerr << e.what() << '\n';
        return EXIT_FAILURE;
    }

    try
    {
        server srv(rend);

        srv.accept();
        srv.run();
        srv.wait();

        std::cout << "done\n";
    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }

    /*float aspect = rend.width() / static_cast<float>(rend.height());

    rend.cam.perspective(45.0f * constants::degrees_to_radians<float>(), aspect, 0.001f, 1000.0f);
    rend.cam.view_all( rend.bbox );

    rend.add_manipulator( std::make_shared<arcball_manipulator>(rend.cam, mouse::Left) );
    rend.add_manipulator( std::make_shared<pan_manipulator>(rend.cam, mouse::Middle) );
    // Additional "Alt + LMB" pan manipulator for setups w/o middle mouse button
    rend.add_manipulator( std::make_shared<pan_manipulator>(rend.cam, mouse::Left, keyboard::Alt) );
    rend.add_manipulator( std::make_shared<zoom_manipulator>(rend.cam, mouse::Right) );

    rend.event_loop();*/
}
