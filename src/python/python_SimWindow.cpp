#include <boost/bind.hpp>
#include <boost/python.hpp>
#include <boost/thread.hpp>
#include <dart/simulation/simulation.h>
#include <dart/gui/gui.h>

using ::dart::gui::SimWindow;
using ::dart::simulation::World;

static SimWindow *gWindow = NULL;
static boost::mutex gMutex;
static boost::condition_variable gCondition;

static void SimWindow_refresh()
{
    if (gWindow) {
        gWindow->refresh();
    }
}

static void SimWindow_main(int width, int height, std::string const &name)
{
    char **argv = NULL;
    int argc = 0;

    std::cout << "glutInit >>>" << std::endl;
    glutInit(&argc, argv);
    std::cout << "glutInit <<<" << std::endl;

    // Notify SimWindow to return.
    {
        boost::mutex::scoped_lock lock(gMutex);
        gWindow = new SimWindow; 
        gWindow->initWindow(width, height, name.c_str());

        gCondition.notify_one();
    }
    // We intentionally leave the mutex locked here.

    // Start the main loop. This will run until glutLeaveMainLoop is called.
    std::cout << "glutMainLoop >>>" << std::endl;
    glutIdleFunc(&SimWindow_refresh);
    glutMainLoop();
    std::cout << "glutMainLoop <<<" << std::endl;
}

static SimWindow *SimWindow_constructor(int width, int height,
                                        std::string const &name)
{
    boost::thread render_thread(
        boost::bind(&SimWindow_main, width, height, name)
    );

    // Wait for OpenGL to initialize.
    std::cout << "waiting >>>" << std::endl;
    {
        boost::mutex::scoped_lock lock(gMutex);
        while (!gWindow) {
            gCondition.wait(lock);
        }
    }
    std::cout << "waiting <<<" << std::endl;

    return gWindow;
}

static World *SimWindow_get_world(SimWindow *window)
{
    throw std::runtime_error("get_world() is not implemented.");
}

void python_SimWindow()
{
    using namespace ::boost::python;

    class_<SimWindow>("SimWindow", no_init)
        .def("__init__", make_constructor(&SimWindow_constructor))
        .add_property("world", 
            make_function(&SimWindow_get_world,
                          return_value_policy<manage_new_object>()),
            &SimWindow::setWorld)
        .def("draw_arrow_2d", &::dart::gui::drawArrow2D)
        .staticmethod("draw_arrow_2d")
        .def("draw_arrow_3d", &::dart::gui::drawArrow3D)
        .staticmethod("draw_arrow_3d")
        ;
}
