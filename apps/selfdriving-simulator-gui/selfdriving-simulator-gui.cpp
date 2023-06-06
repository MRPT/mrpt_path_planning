/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2022 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#include <mpp/algos/CostEvaluator.h>
#include <mpp/algos/CostEvaluatorCostMap.h>
#include <mpp/algos/NavEngine.h>
#include <mpp/algos/TPS_Astar.h>
#include <mpp/algos/viz.h>
#include <mpp/data/Waypoints.h>
#include <mpp/interfaces/MVSIM_VehicleInterface.h>
#include <mpp/interfaces/VehicleMotionInterface.h>
#include <mrpt/3rdparty/tclap/CmdLine.h>
#include <mrpt/config/CConfigFile.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/core/lock_helper.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/math/TLine3D.h>
#include <mrpt/math/TObject3D.h>
#include <mrpt/opengl/CDisk.h>
#include <mrpt/system/CRateTimer.h>
#include <mrpt/system/os.h>  // plugins
#include <mrpt/version.h>
#include <mvsim/Comms/Server.h>
#include <mvsim/World.h>
#include <mvsim/WorldElements/OccupancyGridMap.h>
#include <mvsim/mvsim_version.h>

#include <thread>

#if MVSIM_MAJOR_VERSION > 0 || MVSIM_MINOR_VERSION > 4 || \
    MVSIM_PATCH_VERSION >= 2
#define MVSIM_HAS_POINTCLOUD
#endif

#ifdef MVSIM_HAS_POINTCLOUD
#include <mvsim/WorldElements/PointCloud.h>
#endif

TCLAP::CmdLine cmd(
    "selfdriving-simulator-gui", ' ', "version", false /* no --help */);

TCLAP::ValueArg<std::string> argVerbosity(
    "v", "verbose", "Verbosity level for path planner", false, "INFO",
    "ERROR|WARN|INFO|DEBUG", cmd);

TCLAP::ValueArg<std::string> argVerbosityMVSIM(
    "", "verbose-mvsim", "Verbosity level for the mvsim subsystem", false,
    "INFO", "ERROR|WARN|INFO|DEBUG", cmd);

TCLAP::ValueArg<std::string> arg_config_file_section(
    "", "config-section",
    "If loading from an INI file, the name of the section to load", false,
    "SelfDriving", "SelfDriving", cmd);

TCLAP::ValueArg<std::string> argMvsimFile(
    "s", "simul-file", "MVSIM XML file", true, "xxx.xml", "World XML file",
    cmd);

TCLAP::ValueArg<std::string> argVehicleInterface(
    "", "vehicle-interface-class",
    "Class name to use (Default: 'mpp::MVSIM_VehicleInterface')", false,
    "mpp::MVSIM_VehicleInterface", "Class name to use for vehicle interface",
    cmd);

TCLAP::ValueArg<std::string> argTargetApproachController(
    "", "approach-controller-class",
    "Class name to use as target approach controller (Default: none)", false,
    "", "Class name to use for approach controller", cmd);

TCLAP::ValueArg<std::string> arg_ptgs_file(
    "p", "ptg-config", "Input .ini file with PTG definitions.", true, "",
    "ptgs.ini", cmd);

TCLAP::ValueArg<std::string> arg_planner_yaml_file(
    "", "planner-parameters", "Input .yaml file with planner parameters", false,
    "", "tps-astar.yaml", cmd);

TCLAP::ValueArg<std::string> arg_cost_prefer_waypoints_yaml_file(
    "", "prefer-waypoints-parameters",
    "Input .yaml file with costmap parameters", false, "",
    "cost-prefer-waypoints.yaml", cmd);

TCLAP::ValueArg<std::string> arg_cost_global_yaml_file(
    "", "global-costmap-parameters",
    "Input .yaml file with global obstacle points costmap parameters", false,
    "", "points-costmap.yaml", cmd);

TCLAP::ValueArg<std::string> arg_cost_local_yaml_file(
    "", "local-costmap-parameters",
    "Input .yaml file with local obstacle points costmap parameters", false, "",
    "points-costmap.yaml", cmd);

TCLAP::ValueArg<std::string> arg_nav_engine_yaml_file(
    "", "nav-engine-parameters",
    "Input .yaml file with parameters for NavEngine", false, "",
    "nav-engine.yaml", cmd);

TCLAP::ValueArg<std::string> arg_waypoints_yaml_file(
    "", "waypoints", "Input .yaml file with waypoints", false, "",
    "waypoints.yaml", cmd);

TCLAP::ValueArg<std::string> arg_plugins(
    "", "plugins",
    "Optional plug-in libraries to load, for externally-defined PTGs", false,
    "", "mylib.so", cmd);

std::shared_ptr<mvsim::Server> server;

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

static void                mvsim_server_thread_update_GUI(GUI_ThreadParams& tp);
mvsim::World::TGUIKeyEvent gui_key_events;
std::mutex                 gui_key_events_mtx;
std::string                msg2gui;

// ======= Self Drive status ===================
struct SelfDrivingThreadParams : public CommonThreadParams
{
};

struct SelfDrivingStatus
{
    SelfDrivingStatus() = default;

    mpp::NavEngine navigator;

    mpp::VisualizationOptions vizOpts;

    mpp::WaypointSequence       waypts;
    mpp::WaypointStatusSequence wayptsStatus;

    SelfDrivingThreadParams sdThreadParams;
    std::thread             selfDrivingThread;
};

std::shared_ptr<SelfDrivingStatus> sd;

static void selfdriving_run_thread(SelfDrivingThreadParams& params);
static void on_do_single_path_planning(
    mvsim::World& world, const mpp::SE2_KinState& stateStart,
    const mpp::SE2orR2_KinState& stateGoal);

// ======= End Self Drive status ===================

static mrpt::maps::CSimplePointsMap::Ptr world_to_static_obstacle_points(
    mvsim::World& world)
{
    auto obsPts = mrpt::maps::CSimplePointsMap::Create();

    world.runVisitorOnWorldElements([&](mvsim::WorldElementBase& we) {
        if (auto grid = dynamic_cast<mvsim::OccupancyGridMap*>(&we); grid)
        {  // get grid occupied cells:
            mrpt::maps::CSimplePointsMap pts;
            grid->getOccGrid().getAsPointCloud(pts);
            obsPts->insertAnotherMap(&pts, mrpt::poses::CPose3D::Identity());
        }
#ifdef MVSIM_HAS_POINTCLOUD
        if (auto pc = dynamic_cast<mvsim::PointCloud*>(&we);
            pc && pc->getPoints())
        {
            obsPts->insertAnotherMap(
                pc->getPoints().get(), mrpt::poses::CPose3D::Identity());
        }
#endif
    });
    world.runVisitorOnBlocks([&](mvsim::Block& b) {
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
    // initialize the NavEngine
    // --------------------------------------------------------
    // sd->navigator.config_.multitarget_look_ahead = 2;

    sd->navigator.setMinLoggingLevel(
        mrpt::typemeta::TEnumType<mrpt::system::VerbosityLevel>::name2value(
            argVerbosity.getValue()));

    // Load PTGs:
    {
        mrpt::config::CConfigFile cfg(arg_ptgs_file.getValue());
        sd->navigator.config_.ptgs.initFromConfigFile(
            cfg, arg_config_file_section.getValue());
    }

    // Obstacle source:
    auto obsPts = world_to_static_obstacle_points(world);
    sd->navigator.config_.globalMapObstacleSource =
        mpp::ObstacleSource::FromStaticPointcloud(obsPts);

    sd->navigator.logFmt(
        mrpt::system::LVL_DEBUG,
        "[prepare_selfdriving] Initializing globalMapObstacleSource with "
        "%u points",
        static_cast<unsigned int>(obsPts->size()));

    // Vehicle interface:
    if (argVehicleInterface.isSet())
    {
        const auto name = argVehicleInterface.getValue();
        auto       obj  = mrpt::rtti::classFactory(name);
        ASSERTMSG_(
            obj, mrpt::format("Unregistered class name '%s'", name.c_str()));

        sd->navigator.config_.vehicleMotionInterface =
            std::dynamic_pointer_cast<mpp::VehicleMotionInterface>(obj);
        ASSERTMSG_(
            sd->navigator.config_.vehicleMotionInterface,
            mrpt::format(
                "Class '%s' seems not to implement the expected interface "
                "'mpp::VehicleMotionInterface'",
                name.c_str()));

        sd->navigator.config_.vehicleMotionInterface->setMinLoggingLevel(
            world.getMinLoggingLevel());
    }
    else
    {
        // Default:
        auto sim = std::make_shared<mpp::MVSIM_VehicleInterface>();
        sd->navigator.config_.vehicleMotionInterface = sim;

        sd->navigator.config_.vehicleMotionInterface->setMinLoggingLevel(
            world.getMinLoggingLevel());

        // connect now:
        sim->connect();
    }

    // target approach controller:
    if (argTargetApproachController.isSet())
    {
        const auto name = argTargetApproachController.getValue();
        auto       obj  = mrpt::rtti::classFactory(name);
        ASSERTMSG_(
            obj, mrpt::format("Unregistered class name '%s'", name.c_str()));

        sd->navigator.config_.targetApproachController =
            std::dynamic_pointer_cast<mpp::TargetApproachController>(obj);
        ASSERTMSG_(
            sd->navigator.config_.targetApproachController,
            mrpt::format(
                "Class '%s' seems not to implement the expected interface "
                "'mpp::TargetApproachController'",
                name.c_str()));

        sd->navigator.config_.targetApproachController->setMinLoggingLevel(
            sd->navigator.getMinLoggingLevel());
    }

    if (arg_planner_yaml_file.isSet())
    {
        sd->navigator.config_.plannerParams =
            mpp::TPS_Astar_Parameters::FromYAML(
                mrpt::containers::yaml::FromFile(
                    arg_planner_yaml_file.getValue()));
    }

    if (arg_cost_global_yaml_file.isSet())
    {
        sd->navigator.config_.globalCostParameters =
            mpp::CostEvaluatorCostMap::Parameters::FromYAML(
                mrpt::containers::yaml::FromFile(
                    arg_cost_global_yaml_file.getValue()));
    }

    if (arg_cost_local_yaml_file.isSet())
    {
        sd->navigator.config_.localCostParameters =
            mpp::CostEvaluatorCostMap::Parameters::FromYAML(
                mrpt::containers::yaml::FromFile(
                    arg_cost_local_yaml_file.getValue()));
    }

    if (arg_cost_prefer_waypoints_yaml_file.isSet())
    {
        sd->navigator.config_.preferWaypointsParameters =
            mpp::CostEvaluatorPreferredWaypoint::Parameters::FromYAML(
                mrpt::containers::yaml::FromFile(
                    arg_cost_prefer_waypoints_yaml_file.getValue()));
    }

    if (arg_nav_engine_yaml_file.isSet())
    {
        sd->navigator.config_.loadFrom(mrpt::containers::yaml::FromFile(
            arg_nav_engine_yaml_file.getValue()));
    }

    // all mandaroty fields filled in now:
    sd->navigator.initialize();

    sd->selfDrivingThread =
        std::thread(&selfdriving_run_thread, std::ref(sd->sdThreadParams));

    // Load example/test waypoints?
    // --------------------------------------------------------
    if (arg_waypoints_yaml_file.isSet())
    {
        sd->waypts =
            mpp::WaypointSequence::FromYAML(mrpt::containers::yaml::FromFile(
                arg_waypoints_yaml_file.getValue()));
    }
}

int launchSimulation()
{
    using namespace mvsim;

    sd = std::make_shared<SelfDrivingStatus>();

    const auto sXMLfilename = argMvsimFile.getValue();

    // Start network server:
    commonLaunchServer();

    auto world = std::make_shared<mvsim::World>();

    world->setMinLoggingLevel(
        mrpt::typemeta::TEnumType<mrpt::system::VerbosityLevel>::name2value(
            argVerbosityMVSIM.getValue()));

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

        if (thread_params.isClosing()) do_exit = true;

    }  // end while()

    // Close GUI thread:
    thread_params.closing(true);
    if (thGUI.joinable()) thGUI.join();

    // Close selfdriving thread:
    sd->sdThreadParams.closing(true);
    if (sd->selfDrivingThread.joinable()) sd->selfDrivingThread.join();

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

// Add selfdriving window
void prepare_selfdriving_window(
    const mrpt::gui::CDisplayWindowGUI::Ptr& gui,
    std::shared_ptr<mvsim::World>            world)
{
    ASSERT_(gui);

    auto lck = mrpt::lockHelper(gui->background_scene_mtx);

    // navigator 3D visualization interface:
    sd->navigator.config_.on_viz_pre_modify = [world, &gui]() {
        world->guiUserObjectsMtx_.lock();
        gui->background_scene_mtx.lock();
    };
    sd->navigator.config_.on_viz_post_modify = [world, &gui]() {
        world->guiUserObjectsMtx_.unlock();
        gui->background_scene_mtx.unlock();
    };

    // prepare custom gl objects for selfdriving lib:
    {
        auto lckgui               = mrpt::lockHelper(world->guiUserObjectsMtx_);
        world->guiUserObjectsViz_ = mrpt::opengl::CSetOfObjects::Create();
        world->guiUserObjectsViz_->setName("gui_user_objects_viz");

        sd->navigator.config_.vizSceneToModify = world->guiUserObjectsViz_;
    }

#if MRPT_VERSION >= 0x211
    nanogui::Window* w = gui->createManagedSubWindow("SelfDriving");
#else
    nanogui::Window* w = new nanogui::Window(gui.get(), "SelfDriving");
#endif

    w->setPosition({5, 220});
    w->setLayout(new nanogui::BoxLayout(
        nanogui::Orientation::Vertical, nanogui::Alignment::Fill));
    w->setFixedWidth(260);

    auto tab = w->add<nanogui::TabWidget>();

    std::vector<nanogui::Widget*> tabs = {
        tab->createTab("Waypoints nav"),
        tab->createTab("Viz"),
        tab->createTab("Single A*"),
    };

    tab->setActiveTab(0);

    for (auto t : tabs)
        t->setLayout(new nanogui::BoxLayout(
            nanogui::Orientation::Vertical, nanogui::Alignment::Fill, 3, 3));

    const int pnWidth = 240, pnHeight = 270;

    std::vector<nanogui::VScrollPanel*> vscrolls;
    for (auto t : tabs) vscrolls.emplace_back(t->add<nanogui::VScrollPanel>());

    // vscroll should only have *ONE* child.
    // this is what `wrapper` is for
    std::vector<nanogui::Widget*> wrappers;

    for (auto vs : vscrolls)
    {
        vs->setFixedSize({pnWidth, pnHeight});
        auto wr = vs->add<nanogui::Widget>();
        wr->setLayout(new nanogui::GridLayout(
            nanogui::Orientation::Horizontal, 1 /*columns */,
            nanogui::Alignment::Fill, 3, 3));

        wr->setFixedSize({pnWidth - 7, pnHeight});
        wrappers.emplace_back(wr);
    }

    // -----------------------------------------
    // High-level waypoints-based navigator
    // -----------------------------------------
    nanogui::Label* lbNavStatus = nullptr;
    {
        auto pnNav        = wrappers.at(0);
        auto lbWaypsCount = pnNav->add<nanogui::Label>("");

        lbWaypsCount->setCaption(mrpt::format(
            "Number of wps: %u",
            static_cast<unsigned int>(sd->waypts.waypoints.size())));

        // custom 3D objects
        auto glWaypoints = mrpt::opengl::CSetOfObjects::Create();
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

        lbNavStatus =
            pnNav->add<nanogui::Label>("Nav Status:                    ");

        auto btnReq = pnNav->add<nanogui::Button>("requestNavigation()");
        btnReq->setCallback([world]() {
            // Update global obstacles, in case the MVSIM world has changed:
            auto obsPts = world_to_static_obstacle_points(*world);
            sd->navigator.config_.globalMapObstacleSource =
                mpp::ObstacleSource::FromStaticPointcloud(obsPts);

            sd->navigator.request_navigation(sd->waypts);
        });

        pnNav->add<nanogui::Button>("suspend()")->setCallback([]() {
            sd->navigator.suspend();
        });
        pnNav->add<nanogui::Button>("resume()")->setCallback([]() {
            sd->navigator.resume();
        });
        pnNav->add<nanogui::Button>("cancel()")->setCallback([]() {
            sd->navigator.cancel();
        });
    }
    const auto lambdaUpdateNavStatus = [lbNavStatus]() {
        const auto state = sd->navigator.current_status();
        lbNavStatus->setCaption(mrpt::format(
            "Nav status: %s",
            mrpt::typemeta::TEnumType<mpp::NavStatus>::value2name(state)
                .c_str()));
    };

    // -------------------------------
    // Single A* planner tab
    // -------------------------------
    {
        const mpp::SE2_KinState dummyState;

        auto pnPlanner = wrappers.at(2);
        pnPlanner->add<nanogui::Label>("Start pose:");
        auto edStateStartPose =
            pnPlanner->add<nanogui::TextBox>(dummyState.pose.asString());
        edStateStartPose->setEditable(true);

        pnPlanner->add<nanogui::Label>("Start global vel:");
        auto edStateStartVel =
            pnPlanner->add<nanogui::TextBox>(dummyState.vel.asString());
        edStateStartVel->setEditable(true);

        // custom 3D objects
        auto glTargetSign = mrpt::opengl::CDisk::Create(1.0, 0.8);
        glTargetSign->setColor_u8(0xff, 0x00, 0x00, 0xa0);
        glTargetSign->setName("glTargetSign");
        glTargetSign->setVisibility(false);

        {
            auto lckgui = mrpt::lockHelper(world->guiUserObjectsMtx_);
            world->guiUserObjectsViz_->insert(glTargetSign);
        }

        nanogui::Button* pickBtn = nullptr;

        {
            auto subPn = pnPlanner->add<nanogui::Widget>();
            subPn->setLayout(new nanogui::GridLayout(
                nanogui::Orientation::Horizontal, 2, nanogui::Alignment::Fill,
                2, 2));

            subPn->add<nanogui::Label>("Goal pose:");
            pickBtn = subPn->add<nanogui::Button>("Pick");
        }
        auto edStateGoalPose =
            pnPlanner->add<nanogui::TextBox>(dummyState.pose.asString());
        edStateGoalPose->setEditable(true);

        pickBtn->setCallback([glTargetSign, edStateGoalPose]() {
            activeActionMouseHandler = [glTargetSign,
                                        edStateGoalPose](MouseEvent e) {
                edStateGoalPose->setValue(e.pt.asString());
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
        });

        pnPlanner->add<nanogui::Label>("Goal global vel:");
        auto edStateGoalVel =
            pnPlanner->add<nanogui::TextBox>(dummyState.vel.asString());
        edStateGoalVel->setEditable(true);

        auto btnDoPlan = pnPlanner->add<nanogui::Button>("Do path planning...");
        btnDoPlan->setCallback([=]() {
            try
            {
                mpp::SE2_KinState stateStart;
                stateStart.pose.fromString(edStateStartPose->value());
                stateStart.vel.fromString(edStateStartVel->value());

                mpp::SE2orR2_KinState stateGoal;
                stateGoal.state =
                    mpp::PoseOrPoint::FromString(edStateGoalPose->value());
                stateGoal.vel.fromString(edStateGoalVel->value());

                on_do_single_path_planning(*world, stateStart, stateGoal);
            }
            catch (const std::exception& e)
            {
                std::cerr << e.what() << std::endl;
            }
        });
    }

    // -------------------------------
    // Viz panel
    // -------------------------------
    {
        auto       pnViz = wrappers.at(1);
        const auto cbViewCostmaps =
            pnViz->add<nanogui::CheckBox>("View costmap");
        cbViewCostmaps->setChecked(true);
        cbViewCostmaps->setCallback([](bool /*checked*/) {
            //
        });
    }

    // ----------------------------------
    // Custom event handlers
    // ----------------------------------
    const auto lambdaHandleMouseOperations = [gui, world]() {
        MRPT_START

        static mrpt::math::TPoint3D lastMousePt;
        static bool                 lastLeftClick  = false;
        static bool                 lastRightClick = false;

        const auto& mousePt    = world->gui_mouse_point();
        const auto  screen     = gui->screen();
        const bool  leftClick  = (screen->mouseState() & 0x01) != 0;
        const bool  rightClick = (screen->mouseState() & 0x02) != 0;

        if (lastMousePt != mousePt || leftClick != lastLeftClick ||
            rightClick != lastRightClick)
        {
            MouseEvent e;
            e.pt           = mousePt;
            e.leftBtnDown  = leftClick;
            e.rightBtnDown = rightClick;

            if (activeActionMouseHandler)
            {
                // Make a copy, since the function can modify itself:
                auto act = activeActionMouseHandler;
                act(e);
            }
        }

        lastMousePt    = mousePt;
        lastLeftClick  = leftClick;
        lastRightClick = rightClick;

        MRPT_END
    };

    const auto lambdaCollectSensors = [&]() {
        if (!sd->navigator.config_.vehicleMotionInterface) return;
        if (sd->sdThreadParams.isClosing()) return;

        if (!sd->navigator.config_.localSensedObstacleSource)
            sd->navigator.config_.localSensedObstacleSource =
                std::make_shared<mpp::ObstacleSourceGenericSensor>();

        auto o = std::dynamic_pointer_cast<mpp::ObstacleSourceGenericSensor>(
            sd->navigator.config_.localSensedObstacleSource);
        if (!o) return;

        // handle sensor sources:
        if (auto d = std::dynamic_pointer_cast<mpp::LidarSource>(
                sd->navigator.config_.vehicleMotionInterface);
            d)
        {
            const auto lastLidarFromVeh = d->last_lidar_obs();
            const auto lastLidarInObsSource =
                o->get_stored_sensor_observation();

            if (lastLidarFromVeh &&
                (!lastLidarInObsSource || lastLidarFromVeh->timestamp !=
                                              lastLidarInObsSource->timestamp))
            {
                o->set_sensor_observation(
                    lastLidarFromVeh,
                    mrpt::poses::CPose3D(
                        sd->navigator.config_.vehicleMotionInterface
                            ->get_localization()
                            .pose));
            }
        }
    };

    gui->addLoopCallback(lambdaHandleMouseOperations);

    gui->addLoopCallback(lambdaUpdateNavStatus);

    gui->addLoopCallback(lambdaCollectSensors);

    gui->performLayout();
}

void mvsim_server_thread_update_GUI(GUI_ThreadParams& tp)
{
    while (!tp.isClosing())
    {
        mvsim::World::TUpdateGUIParams guiparams;
        guiparams.msg_lines = msg2gui;

        tp.world->update_GUI(&guiparams);

        static bool firstTime = true;
        if (firstTime)
        {
            tp.world->enqueue_task_to_run_in_gui_thread([&]() {
                prepare_selfdriving_window(tp.world->gui_window(), tp.world);
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

        if (!tp.world->is_GUI_open()) tp.closing(true);

        std::this_thread::sleep_for(std::chrono::milliseconds(25));
    }
}

void selfdriving_run_thread(SelfDrivingThreadParams& params)
{
    double rateHz = 10.0;

    mrpt::system::CRateTimer rate(rateHz);

    while (!params.isClosing())
    {
        try
        {
            sd->navigator.navigation_step();
            rate.sleep();
        }
        catch (const std::exception& e)
        {
            std::cerr << "[selfdriving_run_thread] Exception:" << e.what()
                      << std::endl;
            params.closing(true);
            return;
        }
    }
}

void on_do_single_path_planning(
    mvsim::World& world, const mpp::SE2_KinState& stateStart,
    const mpp::SE2orR2_KinState& stateGoal)
{
    mpp::TPS_Astar    planner;
    mpp::PlannerInput pi;

    // ############################
    // BEGIN: Run path planning
    // ############################
    auto obsPts    = world_to_static_obstacle_points(world);
    auto obstacles = mpp::ObstacleSource::FromStaticPointcloud(obsPts);

    pi.stateStart = stateStart;
    pi.stateGoal  = stateGoal;

    pi.obstacles.push_back(obstacles);

    auto bbox = obstacles->obstacles()->boundingBox();

    // Make sure goal and start are within bbox:
    {
        const auto bboxMargin = mrpt::math::TPoint3Df(1.0, 1.0, .0);
        const auto ptStart    = mrpt::math::TPoint3Df(
            pi.stateStart.pose.x, pi.stateStart.pose.y, 0);
        const auto ptGoal = mrpt::math::TPoint3Df(
            pi.stateGoal.asSE2KinState().pose.x,
            pi.stateGoal.asSE2KinState().pose.y, 0);
        bbox.updateWithPoint(ptStart - bboxMargin);
        bbox.updateWithPoint(ptStart + bboxMargin);
        bbox.updateWithPoint(ptGoal - bboxMargin);
        bbox.updateWithPoint(ptGoal + bboxMargin);
    }

    pi.worldBboxMax = {bbox.max.x, bbox.max.y, M_PI};
    pi.worldBboxMin = {bbox.min.x, bbox.min.y, -M_PI};

    std::cout << "Start state: " << pi.stateStart.asString() << "\n";
    std::cout << "Goal state : " << pi.stateGoal.asString() << "\n";
    std::cout << "Obstacles  : " << obstacles->obstacles()->size()
              << " points\n";
    std::cout << "World bbox: " << pi.worldBboxMin.asString() << " - "
              << pi.worldBboxMax.asString() << "\n";

    // Enable time profiler:
    planner.profiler_().enable(true);

    planner.costEvaluators_.clear();

    mpp::CostEvaluatorCostMap::Parameters cmP;
    cmP.resolution                 = 0.05;
    cmP.preferredClearanceDistance = 1.0;  // [m]

    // cost map for global static obstacles:
    if (!obsPts->empty())
    {
        auto staticCostmap =
            mpp::CostEvaluatorCostMap::FromStaticPointObstacles(
                *obsPts, cmP, pi.stateStart.pose);

        planner.costEvaluators_.push_back(staticCostmap);
    }

    // cost map for observed dynamic obstacles (lidar sensor):
    if (sd->navigator.config_.localSensedObstacleSource)
    {
        const auto obs =
            sd->navigator.config_.localSensedObstacleSource->obstacles();
        if (!obs->empty())
        {
            auto lidarCostmap =
                mpp::CostEvaluatorCostMap::FromStaticPointObstacles(
                    *obs, cmP, pi.stateStart.pose);

            planner.costEvaluators_.push_back(lidarCostmap);
        }
    }

    // Set planner required params:
    if (arg_planner_yaml_file.isSet())
    {
        const auto sFile = arg_planner_yaml_file.getValue();
        const auto c     = mrpt::containers::yaml::FromFile(sFile);
        planner.params_.load_from_yaml(c);
        std::cout << "Loaded these planner params:\n";
        planner.params_.as_yaml().printAsYAML();
    }

    // verbosity level:
    planner.setMinLoggingLevel(mrpt::system::LVL_DEBUG);

    // PTGs config file:
    mrpt::config::CConfigFile cfg(arg_ptgs_file.getValue());
    pi.ptgs.initFromConfigFile(cfg, arg_config_file_section.getValue());

    const mpp::PlannerOutput plan = planner.plan(pi);

    // Visualize:
    mpp::NavEngine::PathPlannerOutput ppo;
    ppo.po             = plan;
    ppo.costEvaluators = planner.costEvaluators_;

    sd->navigator.send_planner_output_to_viz(ppo);

    // ############################
    // END: Run path planning
    // ############################
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
