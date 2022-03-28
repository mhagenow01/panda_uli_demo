#include "PandaController.h"
#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>
#include <franka/robot_state.h>
#include <franka/exception.h>
#include <iostream>
#include <thread>
#include <future>
#include <functional>

using namespace std;

namespace PandaController {
    namespace {
        boost::mutex mutex;

        franka::Gripper *p_gripper = NULL;
        franka::GripperState gripper_state;
        double maxGripperWidth;
        bool isGripperMoving = false;
        bool grasped = false;
    }
    
    franka::GripperState readGripperState() {
        boost::lock_guard<boost::mutex> guard(mutex);
        return gripper_state;
    }

    void writeGripperState() {
        franka::GripperState state = p_gripper->readOnce(); 
        boost::lock_guard<boost::mutex> guard(mutex);
        gripper_state = state;
    }

    void initGripper(const char * ip) {
        isGripperMoving = true;
        try {
            p_gripper = new franka::Gripper(ip);
            franka::GripperState state = p_gripper->readOnce();
            maxGripperWidth = state.max_width;
            p_gripper->move(maxGripperWidth, 0.2);
            PandaController::writeGripperState();
            isGripperMoving = false;
        } catch (franka::CommandException const& e) {
            cout << "Failed to initialize gripper" << endl;
        }
        catch ( const std::exception& e ){
             cout << "Failed to initialize gripper (generic)" << endl;
        }
        
    }

    void graspObj(function<void ()> onGrasp) {
        try {
            p_gripper->stop();
            isGripperMoving = true;
            p_gripper->grasp(maxGripperWidth/2,0.2,40,0.5,0.5);
            isGripperMoving = false;
            grasped = true;
            PandaController::writeGripperState();
            if (onGrasp != NULL) onGrasp();
        } catch (franka::Exception const& e) {
            cout << e.what() << endl;
            isGripperMoving = false;
        }
    }

    void graspObject(function<void ()> onGrasp){
        if(isGripperMoving || grasped) {
            return;
        }
        thread(graspObj, onGrasp).detach();
    }
    void releaseObj(function<void ()> onRelease) {
        try {
            p_gripper->stop();
            isGripperMoving = true;
            p_gripper->move(maxGripperWidth,0.2);
            isGripperMoving = false;
            grasped = false;
            PandaController::writeGripperState();
            if (onRelease != NULL) onRelease();
        } catch (franka::Exception const& e) {
            cout << e.what() << endl;
        }
    }

    void releaseObject(function<void ()> onRelease) {
        if(isGripperMoving || !grasped) {
            return;
        }
        thread(releaseObj, onRelease).detach();
    }

    void toggleGrip(function<void ()> onToggle) {
        if (isGripperMoving){
            return;
        }
        if (grasped) {
            releaseObject(onToggle);
        } else {
            graspObject(onToggle);
        }
    }
}