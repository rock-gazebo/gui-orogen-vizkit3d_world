/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"

using namespace std;
using namespace vizkit3d_world;

const string Task::JOINTS_CMD_POSFIX = ":joints_cmd";

Task::Task(string const& name) :
        TaskBase(name)
{
}

Task::Task(string const& name, RTT::ExecutionEngine* engine) :
        TaskBase(name, engine)
{
}

Task::~Task() {
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.
bool Task::configureHook() {
    //create an instance from Vizkit3dWorld
    if (_widgets.get() < 1) {
        throw runtime_error("There must be at least one instance of Vizkit3dWorld");
    }

    if (!TaskBase::configureHook()) {
        return false;
    }

    //Initialize vizkit3d world
    //this method initialize a thread with event loop
    setupJointsPorts();

    return true;
}

void Task::configureUI() {
    for (int i = 0; i < _widgets.get(); ++i) {
        Vizkit3dWorld* vizkit3dWorld = new Vizkit3dWorld(_world_file_path.value(),
                                        _model_paths.value(),
                                        _ignored_models.get(),
                                        _width.get(),
                                        _height.get(),
                                        60,
                                        0.01,
                                        1000);

        vizkit3dWorlds.push_back(vizkit3dWorld);
    }
    vizkit3dWorld = vizkit3dWorlds[0];
}

bool Task::startHook() {

    if (!TaskBase::startHook()) {
        return false;
    }

    return true;
}

void Task::updateHook() {
    TaskBase::updateHook();
}


void Task::updateUI() {
    updateJoints();
    updatePose();
}

void Task::errorHook() {
    TaskBase::errorHook();
}

void Task::errorUI() {
}

void Task::stopHook() {
    TaskBase::stopHook();
}

void Task::cleanupHook() {
    releaseJointsPorts();

    TaskBase::cleanupHook();
}

void Task::cleanupUI() {
    for (auto vizkit3dWorld: vizkit3dWorlds) {
        delete vizkit3dWorld;
        vizkit3dWorld = nullptr;
    }
    vizkit3dWorlds.clear();
}

void Task::setupJointsPorts()
{
    //get a map with robot models
    vizkit3d_world::RobotVizMap robotVizMap = vizkit3dWorlds[0]->getRobotVizMap();

    //create a input port to receive joints state to each model
    for (vizkit3d_world::RobotVizMap::iterator it = robotVizMap.begin();
        it != robotVizMap.end();
        it++)
    {
        //set the joint name
        //joint name is model name concatenate with posfix ":joints_cmd"
        string nameCmd = it->first + JOINTS_CMD_POSFIX;

        RTT::InputPort<base::samples::Joints>* portIn =
            new RTT::InputPort<base::samples::Joints>(nameCmd);

        //this map stores the original model name and the port name is the key
        mapModel.insert(make_pair(nameCmd, it->first));
        //this map stores the input port and the port name is the key
        mapPorts.insert(make_pair(nameCmd, portIn));

        ports()->addEventPort(*portIn);
    }
}

void Task::releaseJointsPorts(){

    //remove ports from task and delete object from memory
    for (map<string,  RTT::base::PortInterface*>::iterator it  = mapPorts.begin();
             it != mapPorts.end();
             it++)
    {
        ports()->removePort(it->first);
        delete it->second;
    }

    mapPorts.clear();
    mapModel.clear();
}

void Task::updateJoints() {

    //iterate each model store and update joint state
    //read information from input port and send this information to vizkit3d model
    for (map<string, string>::iterator it  = mapModel.begin();
         it != mapModel.end();
         it++)
    {

        string portName = it->first;
        string modelName = it->second;

        RTT::InputPort<base::samples::Joints>* jointCmd =
            dynamic_cast<RTT::InputPort<base::samples::Joints>*>(mapPorts[portName]);

        base::samples::Joints joints;
        for (auto& vizkit3dWorld : vizkit3dWorlds) {
            while (jointCmd->readNewest(joints) == RTT::NewData)
            {
                //set joint states
                vizkit3dWorld->setJoints(modelName, joints);
            }
        }
    }
}

void Task::updatePose() {
    //set model position using vizkit3d transformation
    //read a pose from input port and send perform the transformation
    base::samples::RigidBodyState pose;
    for (auto& vizkit3dWorld : vizkit3dWorlds) {
        while (_pose_cmd.read(pose) == RTT::NewData) {
            vizkit3dWorld->setTransformation(pose);
        }
    }
}

