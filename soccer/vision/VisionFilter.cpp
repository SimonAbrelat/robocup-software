#include "VisionFilter.hpp"

#include <iostream>

#include <Constants.hpp>
#include <Robot.hpp>

#include "vision/util/VisionFilterConfig.hpp"

VisionFilter::VisionFilter() {
    threadEnd.store(false, std::memory_order::memory_order_seq_cst);

    // Have to be careful so the entire initialization list
    // is created before the thread starts
    worker = std::thread(&VisionFilter::updateLoop, this);
}

VisionFilter::~VisionFilter() {
    // Signal end of thread
    threadEnd.store(true, std::memory_order::memory_order_seq_cst);

    // Wait for it to die
    worker.join();
}

void VisionFilter::addFrames(const std::vector<CameraFrame>& frames) {
    std::lock_guard<std::mutex> lock(frameLock);
    frameBuffer.insert(frameBuffer.end(), frames.begin(), frames.end());
}

void VisionFilter::fillBallState(Context* context) {
    std::lock_guard<std::mutex> lock(worldLock);
    const WorldBall& wb = world.getWorldBall();

    if (wb.getIsValid()) {
        context->state.ball.valid = true;
        context->state.ball.pos = wb.getPos();
        context->state.ball.vel = wb.getVel();
        context->state.ball.time = wb.getTime();
    } else {
        context->state.ball.valid = false;
    }
}

void VisionFilter::fillRobotState(Context* context, bool usBlue) {
    std::lock_guard<std::mutex> lock(worldLock);
    auto& ourWorldRobot = usBlue ? world.getRobotsBlue() : world.getRobotsYellow();
    auto& oppWorldRobot = usBlue ? world.getRobotsYellow() : world.getRobotsBlue();

    // Fill our robots
    for (int i = 0; i < Num_Shells; i++) {
        OurRobot* robot = context->state.self.at(i);
        WorldRobot& wr = ourWorldRobot.at(i);

        // TODO(Simon): add inputs from Context to the world robot
        // Use this for the approx commanded position
        MotionSetpoint& setpoint = context->motion_setpoints[robot->shell()];

        RobotState robot_state;
        robot_state.visible = wr.getIsValid();
        robot_state.velocity_valid = wr.getIsValid();

        if (wr.getIsValid()) {
            wr.setCommand(Geometry2d::Twist(setpoint.xvelocity, setpoint.yvelocity, setpoint.avelocity));
            robot_state.pose = Geometry2d::Pose(wr.getPos(), wr.getTheta());
            robot_state.velocity =
                Geometry2d::Twist(wr.getVel(), wr.getOmega());
            robot_state.timestamp = wr.getTime();
        }

        robot->mutable_state() = robot_state;
    }

    // Fill opp robots
    for (int i = 0; i < Num_Shells; i++) {
        OpponentRobot* robot = context->state.opp.at(i);
        const WorldRobot& wr = oppWorldRobot.at(i);

        RobotState robot_state;
        robot_state.visible = wr.getIsValid();
        robot_state.velocity_valid = wr.getIsValid();

        if (wr.getIsValid()) {
            robot_state.pose = Geometry2d::Pose(wr.getPos(), wr.getTheta());
            robot_state.velocity =
                Geometry2d::Twist(wr.getVel(), wr.getOmega());
            robot_state.timestamp = wr.getTime();
        }

        robot->mutable_state() = robot_state;
    }
}

void VisionFilter::updateLoop() {
    while (!threadEnd.load(std::memory_order::memory_order_seq_cst)) {
        RJ::Time start = RJ::now();

        {
            // Do update with whatever is in frame buffer
            std::lock_guard<std::mutex> lock1(frameLock);
            {
                std::lock_guard<std::mutex> lock2(worldLock);

                if (frameBuffer.size() > 0) {
                    world.updateWithCameraFrame(RJ::now(), frameBuffer);
                    frameBuffer.clear();
                } else {
                    world.updateWithoutCameraFrame(RJ::now());
                }
            }
        }

        // Wait for the correct loop timings
        RJ::Seconds diff = RJ::now() - start;
        RJ::Seconds sleepLeft = RJ::Seconds(*VisionFilterConfig::vision_loop_dt) - diff;

        if (diff > RJ::Seconds(0)) {
            std::this_thread::sleep_for(sleepLeft);
        } else {
            std::cout << "WARNING : Filter is not running fast enough" << std::endl;
        }
    }
}
