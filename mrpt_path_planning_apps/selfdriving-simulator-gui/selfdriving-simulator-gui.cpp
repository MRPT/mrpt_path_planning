/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2026 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#include <mpp/algos/CostEvaluatorCostMap.h>
#include <mpp/algos/TPS_Astar.h>
#include <mpp/data/Waypoints.h>
#include <mrpt/config/CConfigFile.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/core/lock_helper.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/math/TLine3D.h>
#include <mrpt/math/TObject3D.h>
#include <mrpt/system/CRateTimer.h>
#include <mrpt/system/os.h>  // plugins
#include <mrpt/version.h>
#include <mrpt/viz/CDisk.h>
#include <mvsim/Comms/Server.h>
#include <mvsim/World.h>
#include <mvsim/WorldElements/OccupancyGridMap.h>
#include <mvsim/WorldElements/PointCloud.h>
#include <mvsim/mvsim_version.h>

#include <CLI/CLI.hpp>
#include <thread>

#include "MVSIM_VehicleInterface.h"
#include "WaypointNavigator.h"

static CLI::App app{"selfdriving-simulator-gui"};

static std::string argVerbosity{"INFO"};
static std::string argVerbosityMVSIM{"INFO"};
static std::string arg_config_file_section{"SelfDriving"};
static std::string argMvsimFile{MPP_APPS_SHARE_DIR "/mvsim-demo.xml"};
static std::string arg_ptgs_file;
static std::string arg_planner_yaml_file;
static bool        arg_planner_yaml_file_set{false};
static std::string arg_cost_global_yaml_file;
static bool        arg_cost_global_yaml_file_set{false};
static std::string arg_follower_yaml_file;
static bool        arg_follower_yaml_file_set{false};
static std::string arg_waypoints_yaml_file;
static bool        arg_waypoints_yaml_file_set{false};
static std::string arg_plugins;
static bool        arg_plugins_set{false};

std::shared_ptr<mvsim::Server> server;

void commonLaunchServer()
{
    ASSERT_(!server);

    // Start network server:
    server = std::make_shared<mvsim::Server>();

    server->setMinLoggingLevel(
        mrpt::typemeta::TEnumType<mrpt::system::VerbosityLevel>::name2value(
            argVerbosityMVSIM));

    server->start();
}

struct CommonThreadParams
{
    std::mutex closingMtx;

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

struct GUI_ThreadParams : public CommonThreadParams
{
    std::shared_ptr<mvsim::World> world;
};

static void               mvsim_server_thread_update_GUI(GUI_ThreadParams& tp);
mvsim::World::GUIKeyEvent gui_key_events;
std::mutex                gui_key_events_mtx;
std::string               msg2gui;

// ======= Self Drive status ===================
struct SelfDrivingStatus
{
    SelfDrivingStatus() = default;

    mpp::WaypointNavigator navigator;
    mpp::WaypointSequence  waypts;
};

std::shared_ptr<SelfDrivingStatus> sd;

// ======= End Self Drive status ===================

static mrpt::maps::CSimplePointsMap::Ptr world_to_static_obstacle_points(
    mvsim::World& world)
{
    auto obsPts = mrpt::maps::CSimplePointsMap::Create();

    world.runVisitorOnWorldElements(
        [&](mvsim::WorldElementBase& we)
        {
            if (auto grid = dynamic_cast<mvsim::OccupancyGridMap*>(&we); grid)
            {  // get grid occupied cells:
                mrpt::maps::CSimplePointsMap pts;
                grid->getOccGrid().getAsPointCloud(pts);
                obsPts->insertAnotherMap(
                    &pts, mrpt::poses::CPose3D::Identity());
            }
            if (auto pc = dynamic_cast<mvsim::PointCloud*>(&we);
                pc && pc->getPoints())
            {
                obsPts->insertAnotherMap(
                    pc->getPoints().get(), mrpt::poses::CPose3D::Identity());
            }
        });
    world.runVisitorOnBlocks(
        [&](mvsim::Block& b)
        {
            mrpt::maps::CSimplePointsMap pts;
            const auto                   shape             = b.blockShape();
            const double                 minDistBetweenPts = 0.1;
            ASSERT_(!shape.empty());
            for (size_t i = 0; i < shape.size(); i++)
            {
                const size_t ip1 = (i + 1) % shape.size();
                const auto   pt0 = shape.at(i);
                const auto   pt1 = shape.at(ip1);
                // sample:
                const double dist      = (pt1 - pt0).norm();
                const size_t nSamples  = std::ceil(dist / minDistBetweenPts);
                const auto   dirVector = (pt1 - pt0).unitarize();
                for (size_t k = 0; k < nSamples; k++)
                {
                    const auto pt = pt0 + dirVector * k * dist / (nSamples + 1);
                    pts.insertPointFast(pt.x, pt.y, 0);
                }
            }
            obsPts->insertAnotherMap(&pts, mrpt::poses::CPose3D(b.getPose()));
        });

    return obsPts;
}

void prepare_selfdriving(mvsim::World& world)
{
    auto& nav = sd->navigator;
    auto& cfg = nav.config;

    nav.setMinLoggingLevel(
        mrpt::typemeta::TEnumType<mrpt::system::VerbosityLevel>::name2value(
            argVerbosity));

    // Load PTGs and the robot description:
    {
        mrpt::config::CConfigFile c(arg_ptgs_file);
        cfg.ptgs.initFromConfigFile(c, arg_config_file_section);
    }

    // Static obstacles, for planning:
    auto obsPts = world_to_static_obstacle_points(world);
    nav.set_static_obstacles(obsPts);
    nav.logFmt(
        mrpt::system::LVL_DEBUG, "Static obstacles: %u points",
        static_cast<unsigned int>(obsPts->size()));

    // Vehicle interface, for the first robot in the world:
    const auto& vehicles = world.getListOfVehicles();
    if (vehicles.empty())
    {
        THROW_EXCEPTION_FMT(
            "The world '%s' has no vehicle to navigate.", argMvsimFile.c_str());
    }
    const std::string robotName = vehicles.begin()->first;
    if (vehicles.size() > 1)
    {
        std::cout << "The world has " << vehicles.size()
                  << " vehicles: navigating with the first one, '" << robotName
                  << "'.\n";
    }

    auto sim = std::make_shared<mpp::MVSIM_VehicleInterface>(robotName);
    sim->setMinLoggingLevel(world.getMinLoggingLevel());
    sim->connect();

    // Lidar observations, straight from the simulator:
    world.registerCallbackOnObservation(
        [sim](
            const mvsim::Simulable&             veh,
            const mrpt::obs::CObservation::Ptr& obs)
        {
            if (veh.getName() == sim->robot_name())
            {
                sim->on_observation(obs);
            }
        });
    cfg.vehicle = sim;
    cfg.lidar   = sim;

    if (arg_planner_yaml_file_set)
    {
        cfg.plannerParams = mpp::TPS_Astar_Parameters::FromYAML(
            mrpt::containers::yaml::FromFile(arg_planner_yaml_file));
    }

    if (arg_cost_global_yaml_file_set)
    {
        cfg.globalCostParams = mpp::CostEvaluatorCostMap::Parameters::FromYAML(
            mrpt::containers::yaml::FromFile(arg_cost_global_yaml_file));
    }

    if (arg_follower_yaml_file_set)
    {
        nav.follower.params.load_from_yaml(
            mrpt::containers::yaml::FromFile(arg_follower_yaml_file));
    }

    nav.start();

    // Load example/test waypoints?
    if (arg_waypoints_yaml_file_set)
    {
        sd->waypts = mpp::WaypointSequence::FromYAML(
            mrpt::containers::yaml::FromFile(arg_waypoints_yaml_file));
    }
}

int launchSimulation()
{
    using namespace mvsim;

    sd = std::make_shared<SelfDrivingStatus>();

    const auto sXMLfilename = argMvsimFile;

    // Start network server:
    commonLaunchServer();

    auto world = std::make_shared<mvsim::World>();

    world->setMinLoggingLevel(
        mrpt::typemeta::TEnumType<mrpt::system::VerbosityLevel>::name2value(
            argVerbosityMVSIM));

    // Load from XML:
    world->load_from_XML_file(sXMLfilename);

    // Attach world as a mvsim communications node:
    world->connectToServer();

    // Prepare selfdriving classes, now that we have the world initialized:
    prepare_selfdriving(*world);

    // Launch GUI thread:
    GUI_ThreadParams thread_params;
    thread_params.world = world;

    std::thread thGUI =
        std::thread(&mvsim_server_thread_update_GUI, std::ref(thread_params));

    // Run simulation:
    const double tAbsInit       = mrpt::Clock::nowDouble();
    bool         do_exit        = false;
    size_t       teleop_idx_veh = 0;  // Index of the vehicle to teleop

    while (!do_exit)
    {
        // was the quit button hit in the GUI?
        if (world->simulator_must_close()) break;

        // Simulation
        // ============================================================
        // Compute how much time has passed to simulate in real-time:
        double tNew          = mrpt::Clock::nowDouble();
        double incrTime      = (tNew - tAbsInit) - world->get_simul_time();
        int    incrTimeSteps = static_cast<int>(
            std::floor(incrTime / world->get_simul_timestep()));

        // Simulate:
        if (incrTimeSteps > 0)
        {  // simulate world:
            world->run_simulation(incrTimeSteps * world->get_simul_timestep());
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        // GUI msgs, teleop, etc.
        // ====================================================

        std::string txt2gui_tmp;
        gui_key_events_mtx.lock();
        mvsim::World::GUIKeyEvent keyevent = gui_key_events;
        gui_key_events_mtx.unlock();

        // Global keys:
        switch (keyevent.keycode)
        {
            case mvsim::World::GUIKeyEvent::KEY_ESCAPE:
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
            const World::VehicleList& vehs = world->getListOfVehicles();
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
#if MVSIM_MAJOR_VERSION > 1 || \
    (MVSIM_MAJOR_VERSION == 1 && MVSIM_MINOR_VERSION >= 2)
                    const mrpt::math::TTwist2D& vel =
                        it_veh->second->getRefVelocityLocal();
#else
                    const mrpt::math::TTwist2D& vel =
                        it_veh->second->getVelocityLocal();
#endif
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
        if (keyevent.keycode != 0) { gui_key_events = {}; }
        gui_key_events_mtx.unlock();

        msg2gui = txt2gui_tmp;  // send txt msgs to show in the GUI

        if (thread_params.isClosing()) do_exit = true;

    }  // end while()

    // Close GUI thread:
    thread_params.closing(true);
    if (thGUI.joinable()) thGUI.join();

    // Stops the navigation threads:
    sd.reset();

    return 0;
}

// ======= GUI status ===================
struct MouseEvent
{
    MouseEvent() = default;

    mrpt::math::TPoint3D pt{0, 0, 0};
    bool                 leftBtnDown  = false;
    bool                 rightBtnDown = false;
    bool                 shiftDown    = false;
    bool                 ctrlDown     = false;
};

using on_mouse_event_callback_t = std::function<void(MouseEvent)>;

on_mouse_event_callback_t activeActionMouseHandler;
// ======= end GUI status ==============

// Adds the selfdriving panel to the mvsim GUI. Returns a function to be called
// periodically from a non-GUI thread, which updates the panel status text and
// collects sensor data.
std::function<void()> prepare_selfdriving_window(
    const std::shared_ptr<mvsim::World>& world)
{
    // navigator 3D visualization interface:
    sd->navigator.config.on_viz_pre_modify = [world]()
    { world->guiUserObjectsMtx_.lock(); };
    sd->navigator.config.on_viz_post_modify = [world]()
    { world->guiUserObjectsMtx_.unlock(); };

    // prepare custom gl objects for selfdriving lib:
    {
        auto lckgui               = mrpt::lockHelper(world->guiUserObjectsMtx_);
        world->guiUserObjectsViz_ = mrpt::viz::CSetOfObjects::Create();
        world->guiUserObjectsViz_->setName("gui_user_objects_viz");

        sd->navigator.config.vizScene = world->guiUserObjectsViz_;
    }

    using mvsim::gui::LiveString;

    mvsim::gui::WindowDescription win;
    win.title = "SelfDriving";
    win.size  = {300, 0};

    // -----------------------------------------
    // High-level waypoints-based navigator
    // -----------------------------------------
    auto lbNavStatus = std::make_shared<LiveString>("Nav status:");
    {
        mvsim::gui::Tab tab;
        tab.title = "Waypoints nav";

        tab.widgets.emplace_back(
            mvsim::gui::Label{std::make_shared<LiveString>(mrpt::format(
                "Number of wps: %u",
                static_cast<unsigned int>(sd->waypts.waypoints.size())))});

        // custom 3D objects
        auto glWaypoints = mrpt::viz::CSetOfObjects::Create();
        glWaypoints->setLocation(0, 0, 0.01);
        glWaypoints->setName("glWaypoints");
        mpp::WaypointsRenderingParams rp;
        // rp.xx = x;

        sd->waypts.getAsOpenglVisualization(*glWaypoints, rp);
        std::cout << "Waypoints:\n" << sd->waypts.getAsText() << std::endl;

        {
            auto lckgui = mrpt::lockHelper(world->guiUserObjectsMtx_);
            world->guiUserObjectsViz_->insert(glWaypoints);
        }

        tab.widgets.emplace_back(mvsim::gui::Label{lbNavStatus});

        tab.widgets.emplace_back(mvsim::gui::Button{
            "requestNavigation()", [world]()
            {
                // Update global obstacles, in case the MVSIM world has changed:
                auto obsPts = world_to_static_obstacle_points(*world);
                sd->navigator.set_static_obstacles(obsPts);

                sd->navigator.request_navigation(sd->waypts);
            }});
        tab.widgets.emplace_back(
            mvsim::gui::Button{"suspend()", []() { sd->navigator.suspend(); }});
        tab.widgets.emplace_back(
            mvsim::gui::Button{"resume()", []() { sd->navigator.resume(); }});
        tab.widgets.emplace_back(
            mvsim::gui::Button{"cancel()", []() { sd->navigator.cancel(); }});

        win.tabs.emplace_back(std::move(tab));
    }

    // -------------------------------
    // Single A* planner tab
    // -------------------------------
    {
        const mpp::SE2_KinState dummyState;

        mvsim::gui::Tab tab;
        tab.title = "Single A*";

        auto edStateStartPose =
            std::make_shared<LiveString>(dummyState.pose.asString());
        auto edStateStartVel =
            std::make_shared<LiveString>(dummyState.vel.asString());
        auto edStateGoalPose =
            std::make_shared<LiveString>(dummyState.pose.asString());
        auto edStateGoalVel =
            std::make_shared<LiveString>(dummyState.vel.asString());

        // custom 3D objects
        auto glTargetSign = mrpt::viz::CDisk::Create(1.0, 0.8);
        glTargetSign->setColor_u8(0xff, 0x00, 0x00, 0xa0);
        glTargetSign->setName("glTargetSign");
        glTargetSign->setVisibility(false);

        {
            auto lckgui = mrpt::lockHelper(world->guiUserObjectsMtx_);
            world->guiUserObjectsViz_->insert(glTargetSign);
        }

        mvsim::gui::Row startRow;
        startRow.widgets.emplace_back(
            mvsim::gui::Label{std::make_shared<LiveString>("Start pose:")});
        startRow.widgets.emplace_back(mvsim::gui::Button{
            "Robot pose", [edStateStartPose]()
            {
                const auto loc =
                    sd->navigator.config.vehicle->get_localization();
                edStateStartPose->set(loc.pose.asString());
            }});
        tab.widgets.emplace_back(std::move(startRow));
        tab.widgets.emplace_back(mvsim::gui::TextBox{"", edStateStartPose, {}});
        tab.widgets.emplace_back(
            mvsim::gui::TextBox{"Start global vel:", edStateStartVel, {}});

        mvsim::gui::Row goalRow;
        goalRow.widgets.emplace_back(
            mvsim::gui::Label{std::make_shared<LiveString>("Goal pose:")});
        goalRow.widgets.emplace_back(mvsim::gui::Button{
            "Pick", [glTargetSign, edStateGoalPose]()
            {
                activeActionMouseHandler =
                    [glTargetSign, edStateGoalPose](MouseEvent e)
                {
                    edStateGoalPose->set(
                        mrpt::math::TPoint2D(e.pt.x, e.pt.y).asString());
                    glTargetSign->setLocation(
                        e.pt + mrpt::math::TVector3D(0, 0, 0.05));
                    glTargetSign->setVisibility(true);

                    // Click -> end mode:
                    if (e.leftBtnDown)
                    {
                        activeActionMouseHandler = {};
                        glTargetSign->setVisibility(false);
                    }
                };
            }});
        tab.widgets.emplace_back(std::move(goalRow));
        tab.widgets.emplace_back(mvsim::gui::TextBox{"", edStateGoalPose, {}});
        tab.widgets.emplace_back(
            mvsim::gui::TextBox{"Goal global vel:", edStateGoalVel, {}});

        // The text boxes are read from the GUI thread, where this runs:
        tab.widgets.emplace_back(mvsim::gui::Button{
            "Do path planning...", [=]()
            {
                try
                {
                    mpp::SE2_KinState stateStart;
                    stateStart.pose.fromString(edStateStartPose->display);
                    stateStart.vel.fromString(edStateStartVel->display);

                    mpp::SE2orR2_KinState stateGoal;
                    stateGoal.state =
                        mpp::PoseOrPoint::FromString(edStateGoalPose->display);
                    stateGoal.vel.fromString(edStateGoalVel->display);

                    sd->navigator.request_single_plan(stateStart, stateGoal);
                }
                catch (const std::exception& e)
                {
                    std::cerr << e.what() << std::endl;
                }
            }});
        tab.widgets.emplace_back(mvsim::gui::Button{
            "Follow the plan", []() { sd->navigator.follow_last_plan(); }});

        win.tabs.emplace_back(std::move(tab));
    }

    world->add_gui_panel(win);

    // ----------------------------------
    // Custom event handlers
    // ----------------------------------

    // Runs in the GUI thread, once per frame:
    world->set_gui_mouse_callback(
        [lastMousePt = mrpt::math::TPoint3D(), lastLeftClick = false,
         lastRightClick = false](const mvsim::gui::MouseState& ms) mutable
        {
            if (lastMousePt != ms.pt || ms.left_down != lastLeftClick ||
                ms.right_down != lastRightClick)
            {
                MouseEvent e;
                e.pt           = ms.pt;
                e.leftBtnDown  = ms.left_down;
                e.rightBtnDown = ms.right_down;

                if (activeActionMouseHandler)
                {
                    // Make a copy, since the function can modify itself:
                    auto act = activeActionMouseHandler;
                    act(e);
                }
            }

            lastMousePt    = ms.pt;
            lastLeftClick  = ms.left_down;
            lastRightClick = ms.right_down;
        });

    return [lbNavStatus]()
    { lbNavStatus->set("Nav status: " + sd->navigator.status_text()); };
}

void mvsim_server_thread_update_GUI(GUI_ThreadParams& tp)
{
    std::function<void()> selfdrivingPeriodicTask;

    while (!tp.isClosing())
    {
        mvsim::World::TUpdateGUIParams guiparams;
        guiparams.msg_lines = msg2gui;

        tp.world->update_GUI(&guiparams);

        // The GUI window is open after the first update_GUI():
        if (!selfdrivingPeriodicTask && tp.world->is_GUI_open())
        {
            selfdrivingPeriodicTask = prepare_selfdriving_window(tp.world);
        }
        if (selfdrivingPeriodicTask) { selfdrivingPeriodicTask(); }

        // Send key-strokes to the main thread:
        if (guiparams.keyevent.keycode != 0)
        {
            gui_key_events_mtx.lock();
            gui_key_events = guiparams.keyevent;
            gui_key_events_mtx.unlock();
        }

        if (!tp.world->is_GUI_open()) tp.closing(true);

        std::this_thread::sleep_for(std::chrono::milliseconds(25));
    }
}

int main(int argc, char** argv)
{
    try
    {
        app.add_option(
            "-v,--verbose", argVerbosity, "Verbosity level for path planner.");
        app.add_option(
            "--verbose-mvsim", argVerbosityMVSIM,
            "Verbosity level for the mvsim subsystem.");
        app.add_option(
            "--config-section", arg_config_file_section,
            "If loading from an INI file, the name of the section to load.");
        app.add_option(
            "-s,--simul-file", argMvsimFile,
            "MVSIM world XML file. The first vehicle in it is the one to "
            "navigate. Default: the demo world.");
        app.add_option(
               "-p,--ptg-config", arg_ptgs_file,
               "Input .ini file with PTG definitions.")
            ->required();
        app.add_option(
            "--planner-parameters", arg_planner_yaml_file,
            "Input .yaml file with planner parameters.");
        app.add_option(
            "--global-costmap-parameters", arg_cost_global_yaml_file,
            "Input .yaml file with global obstacle points costmap parameters.");
        app.add_option(
            "--follower-parameters", arg_follower_yaml_file,
            "Input .yaml file with trajectory follower parameters.");
        app.add_option(
            "--waypoints", arg_waypoints_yaml_file,
            "Input .yaml file with waypoints.");
        app.add_option(
            "--plugins", arg_plugins,
            "Optional plug-in libraries to load, for externally-defined PTGs.");

        CLI11_PARSE(app, argc, argv);

        arg_planner_yaml_file_set = (app.count("--planner-parameters") > 0);
        arg_cost_global_yaml_file_set =
            (app.count("--global-costmap-parameters") > 0);
        arg_follower_yaml_file_set  = (app.count("--follower-parameters") > 0);
        arg_waypoints_yaml_file_set = (app.count("--waypoints") > 0);
        arg_plugins_set             = (app.count("--plugins") > 0);

        if (arg_plugins_set)
        {
            std::string loadErrors;
            if (!mrpt::system::loadPluginModules(arg_plugins, loadErrors))
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
