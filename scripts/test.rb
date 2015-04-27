#
# In this sample test we send an array with the model state
# updateHook read this information and update the model joints and positions
#
require 'orocos'
require 'vizkit'
require 'sdf'
require 'transformer'
require 'transformer/sdf'

include Orocos

Orocos.initialize

path = File.join(Dir.pwd, 'test_data', 'simple.world')


vizkit3d = Vizkit.vizkit3d_widget
main = Qt::Widget.new
mainlayout = Qt::HBoxLayout.new(main)
vizkit3dlayout = Qt::VBoxLayout.new
jointslayout = Qt::VBoxLayout.new

mainlayout.addLayout(vizkit3dlayout)
mainlayout.addLayout(jointslayout)

vizkit3dlayout.addWidget(vizkit3d)

#robot_model is a dependency of this orogen package.
#insert robot_model/test_data folder in the model paths
model_path = ["#{ENV['AUTOPROJ_CURRENT_ROOT']}/gui/robot_model/test_data"]
  
SDF::XML.model_path = model_path 
sdf = SDF::Root.load(path)

Orocos.run 'vizkit3d_world::Task' => 'vizkit3d_world' do
    
    #hash map that stores the model in the world file
    #this hash map is used to count the numbers of model with the same name
    models_map = {}

    #hash map that stores the index of the model state in the array of model states
    index_map = {}
   
    world_task = TaskContext.get 'vizkit3d_world'
    
    #this sample stores an array with models states
    model_states = world_task.model_states.writer.new_sample
   
    #set an array with model paths
    world_task.model_paths = model_path
    
    #path to world file
    world_task.world_file_path = path
    world_task.configure
    world_task.start
    
    cnt = 0
        
    sdf.each_model(recursive: true) do |model|

        
        model_viz = Vizkit.default_loader.RobotVisualization
        
        model_only = model.make_root
        model_viz.loadFromString(model_only.xml.to_s, 'sdf', File.dirname(path))
        plugin_name = model.name              

        #allow create a world files with multiple models with same names
        #create a new name if there is a model with the same name in the models_map
        if models_map.has_key?(model.name) then
            #change model name
            plugin_name = "#{model.name}_#{models_map[model.name]}"
            models_map[model.name] = models_map[model.name] + 1 
        else
            models_map[model.name] = 0
        end
        
        model_viz.setPluginName(plugin_name)
        
        #fill the model initial pose        
        pose = Types::Base::Samples::RigidBodyState.new
        pose.zero!
        pose.time = Time.now
        pose.position = model.pose.translation
        pose.orientation = model.pose.rotation
        
        model_viz.updateData(pose)
        
        override_vel_limits=0
        only_positive=true
        no_effort=true
        no_velocity=true
    
        #create control ui to simulate gazebo data
        ctrl_gui = Vizkit.default_loader.ControlUi
        ctrl_gui.configureUi(override_vel_limits, only_positive, no_effort, no_velocity)
        ctrl_gui.initFromFile(SDF::XML.model_path_from_name(model.name))

        
        # variable to store the state of the model
        # receive the model name, pose and joint states
        state = Types::Vizkit3dWorld::ModelState.s
        state.model_name = plugin_name
        state.pose = pose
                
        
        #this hash map stores the index o state in the array
        index_map[plugin_name] = cnt
        cnt = cnt + 1
        
        model_states << state

        ctrl_gui.connect(SIGNAL('sendSignal()')) do
            joints = ctrl_gui.getJoints()
            state.joints = joints
            
            model_states[index_map[plugin_name]] = state
            
            # I don't know why, but if I try to change only the joints information doesn't work
            # model_states[index_map[plugin_name]].joints = joints
            world_task.model_states.writer.write(model_states)
            model_viz.updateData(joints)
        end
               
        jointslayout.add_widget(ctrl_gui)
        
    end
    
    main.show
    Vizkit.exec    
end

