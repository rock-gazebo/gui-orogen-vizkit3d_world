/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"

using namespace vizkit3d_world;

Task::Task(std::string const& name) :
        TaskBase(name) {
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine) :
        TaskBase(name, engine) {
}

Task::~Task() {
    delete vizkit3dWorld;
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.
bool Task::configureHook() {
    if (!TaskBase::configureHook())
        return false;


    vizkit3dWorld = new Vizkit3dWorld(_world_file_path.value(), _model_paths.value());

    return true;
}
bool Task::startHook() {
    if (!TaskBase::startHook())
        return false;

    vizkit3dWorld->start();

    return true;
}

void Task::updateHook() {
    TaskBase::updateHook();
    std::vector<vizkit3d_world::ModelState> modelStates;

    if (_model_states.read(modelStates) == RTT::NewData){
        std::vector<vizkit3d_world::ModelState>::iterator it;
        for (it = modelStates.begin(); it != modelStates.end(); it++){
            vizkit3dWorld->updateModel(it->model_name, it->pose, it->joints);
        }
    }
}

void Task::errorHook() {
    TaskBase::errorHook();
}

void Task::stopHook() {
    TaskBase::stopHook();

    vizkit3dWorld->stop();

}

void Task::cleanupHook() {
    TaskBase::cleanupHook();
}
