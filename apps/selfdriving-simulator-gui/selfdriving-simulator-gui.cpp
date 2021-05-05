/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2021 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#include <mrpt/3rdparty/tclap/CmdLine.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/core/lock_helper.h>
#include <mrpt/system/os.h>  // plugins
#include <mrpt/version.h>
#include <mvsim/Comms/Server.h>
#include <mvsim/World.h>

#include <rapidxml_utils.hpp>
#include <thread>

static TCLAP::CmdLine cmd(
    "selfdriving-simulator-gui", ' ', "version", false /* no --help */);

static TCLAP::ValueArg<std::string> argVerbosity(
    "v", "verbose", "Verbosity level for path planner", false, "INFO",
    "ERROR|WARN|INFO|DEBUG", cmd);

static TCLAP::ValueArg<std::string> argVerbosityMVSIM(
    "", "verbose-mvsim", "Verbosity level for the mvsim subsystem", false,
    "INFO", "ERROR|WARN|INFO|DEBUG", cmd);

static TCLAP::ValueArg<std::string> argMvsimFile(
    "s", "simul-file", "MVSIM XML file", true, "xxx.xml", "World XML file",
    cmd);

static TCLAP::ValueArg<std::string> arg_ptgs_file(
    "p", "ptg-config", "Input .ini file with PTG definitions.", true, "",
    "ptgs.ini", cmd);

static TCLAP::ValueArg<std::string> arg_planner_yaml_file(
    "", "planner-parameters", "Input .yaml file with planner parameters", false,
    "", "tps-rrtstar.yaml", cmd);

static TCLAP::ValueArg<std::string> arg_plugins(
    "", "plugins",
    "Optional plug-in libraries to load, for externally-defined PTGs", false,
    "", "mylib.so", cmd);

static std::shared_ptr<mvsim::Server> server;

void commonLaunchServer()
{
    ASSERT_(!server);

    // Start network server:
    server = std::make_shared<mvsim::Server>();

    server->setMinLoggingLevel(
        mrpt::typemeta::TEnumType<mrpt::system::VerbosityLevel>::name2value(
            argVerbosityMVSIM.getValue()));

    server->start();
}
struct TThreadParams
{
    mvsim::World* world = nullptr;
    std::mutex    closingMtx;

    TThreadParams() = default;

    bool isClosing()
    {
        closingMtx.lock();
        bool ret = closing_;
        closingMtx.unlock();
        return ret;
    }
    void closing(bool v)
    {
        closingMtx.lock();
        closing_ = v;
        closingMtx.unlock();
    }

   private:
    bool closing_ = false;
};
static void                mvsim_server_thread_update_GUI(TThreadParams& tp);
mvsim::World::TGUIKeyEvent gui_key_events;
std::mutex                 gui_key_events_mtx;
std::string                msg2gui;

int launchSimulation()
{
    using namespace mvsim;

    const auto sXMLfilename = argMvsimFile.getValue();

    // Start network server:
    commonLaunchServer();

    mvsim::World world;

    world.setMinLoggingLevel(
        mrpt::typemeta::TEnumType<mrpt::system::VerbosityLevel>::name2value(
            argVerbosityMVSIM.getValue()));

    // Load from XML:
    rapidxml::file<> fil_xml(sXMLfilename.c_str());
    world.load_from_XML(fil_xml.data(), sXMLfilename.c_str());

    // Attach world as a mvsim communications node:
    world.connectToServer();

    // Launch GUI thread:
    TThreadParams thread_params;
    thread_params.world = &world;
    std::thread thGUI =
        std::thread(&mvsim_server_thread_update_GUI, std::ref(thread_params));

    // Run simulation:
    mrpt::system::CTicTac tictac;
    double                t_old           = tictac.Tac();
    double                REALTIME_FACTOR = 1.0;
    bool                  do_exit         = false;
    size_t                teleop_idx_veh = 0;  // Index of the vehicle to teleop

    while (!do_exit && !mrpt::system::os::kbhit())
    {
        // Simulation
        // ============================================================
        // Compute how much time has passed to simulate in real-time:
        double t_new     = tictac.Tac();
        double incr_time = REALTIME_FACTOR * (t_new - t_old);

        // Just in case the computer is *really fast*...
        if (incr_time >= world.get_simul_timestep())
        {
            // Simulate:
            world.run_simulation(incr_time);

            // t_old_simul = world.get_simul_time();
            t_old = t_new;
        }

        // I could use 10ms here but chono literals are since gcc 4.9.3
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        // GUI msgs, teleop, etc.
        // ====================================================

        std::string txt2gui_tmp;
        gui_key_events_mtx.lock();
        World::TGUIKeyEvent keyevent = gui_key_events;
        gui_key_events_mtx.unlock();

        // Global keys:
        switch (keyevent.keycode)
        {
            case GLFW_KEY_ESCAPE:
                do_exit = true;
                break;
            case '1':
            case '2':
            case '3':
            case '4':
            case '5':
            case '6':
                teleop_idx_veh = keyevent.keycode - '1';
                break;
        };

        {  // Test: Differential drive: Control raw forces
            const World::VehicleList& vehs = world.getListOfVehicles();
            txt2gui_tmp += mrpt::format(
                "Selected vehicle: %u/%u\n",
                static_cast<unsigned>(teleop_idx_veh + 1),
                static_cast<unsigned>(vehs.size()));
            if (vehs.size() > teleop_idx_veh)
            {
                // Get iterator to selected vehicle:
                World::VehicleList::const_iterator it_veh = vehs.begin();
                std::advance(it_veh, teleop_idx_veh);

                // Get speed: ground truth
                {
                    const mrpt::math::TTwist2D& vel =
                        it_veh->second->getVelocityLocal();
                    txt2gui_tmp += mrpt::format(
                        "gt. vel: lx=%7.03f, ly=%7.03f, w= %7.03fdeg/s\n",
                        vel.vx, vel.vy, mrpt::RAD2DEG(vel.omega));
                }
                // Get speed: ground truth
                {
                    const mrpt::math::TTwist2D& vel =
                        it_veh->second->getVelocityLocalOdoEstimate();
                    txt2gui_tmp += mrpt::format(
                        "odo vel: lx=%7.03f, ly=%7.03f, w= %7.03fdeg/s\n",
                        vel.vx, vel.vy, mrpt::RAD2DEG(vel.omega));
                }

                // Generic teleoperation interface for any controller that
                // supports it:
                {
                    ControllerBaseInterface* controller =
                        it_veh->second->getControllerInterface();
                    ControllerBaseInterface::TeleopInput  teleop_in;
                    ControllerBaseInterface::TeleopOutput teleop_out;
                    teleop_in.keycode = keyevent.keycode;
                    controller->teleop_interface(teleop_in, teleop_out);
                    txt2gui_tmp += teleop_out.append_gui_lines;
                }
            }
        }

        // Clear the keystroke buffer
        gui_key_events_mtx.lock();
        if (keyevent.keycode != 0) gui_key_events = World::TGUIKeyEvent();
        gui_key_events_mtx.unlock();

        msg2gui = txt2gui_tmp;  // send txt msgs to show in the GUI

    }  // end while()

    thread_params.closing(true);

    thGUI.join();  // TODO: It could break smth

    return 0;
}

// Add selfdriving window
void prepare_selfdriving_window(const mrpt::gui::CDisplayWindowGUI::Ptr& gui)
{
    auto lck = mrpt::lockHelper(gui->background_scene_mtx);

    ASSERT_(gui);
#if MRPT_VERSION >= 0x211
    nanogui::Window* w = gui->createManagedSubWindow("SelfDriving");
#else
    nanogui::Window* w = new nanogui::Window(gui.get(), "SelfDriving");
#endif

    w->setPosition({5, 220});
    w->setLayout(new nanogui::BoxLayout(
        nanogui::Orientation::Vertical, nanogui::Alignment::Fill));
    w->setFixedWidth(230);

    w->add<nanogui::Label>("XXX");

    gui->performLayout();
}

void mvsim_server_thread_update_GUI(TThreadParams& tp)
{
    while (!tp.isClosing())
    {
        mvsim::World::TUpdateGUIParams guiparams;
        guiparams.msg_lines = msg2gui;

        tp.world->update_GUI(&guiparams);

        static bool firstTime = true;
        if (firstTime)
        {
            tp.world->enqueue_task_to_run_in_gui_thread([&tp]() {
                prepare_selfdriving_window(tp.world->gui_window());
            });
            firstTime = false;
        }

        // Send key-strokes to the main thread:
        if (guiparams.keyevent.keycode != 0)
        {
            gui_key_events_mtx.lock();
            gui_key_events = guiparams.keyevent;
            gui_key_events_mtx.unlock();
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(25));
    }
}

int main(int argc, char** argv)
{
    try
    {
        if (!cmd.parse(argc, argv)) return 1;

        if (arg_plugins.isSet())
        {
            std::string loadErrors;
            if (!mrpt::system::loadPluginModules(
                    arg_plugins.getValue(), loadErrors))
            {
                std::cerr << "Could not load plugins, error: " << loadErrors;
                return 1;
            }
        }

        launchSimulation();
    }
    catch (const std::exception& e)
    {
        std::cerr << "ERROR: " << mrpt::exception_to_str(e);
        return 1;
    }
    return 0;
}
